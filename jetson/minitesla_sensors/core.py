"""四向測距、雷達、視覺的時間與有效性整理。Python 3.8，僅標準函式庫。

車體座標沿用專案定義：車尾原點，X 向前、Y 向右、Z 向上（非 ROS 預設）。
雷達安裝偏角未設定時，只保留雷達原始角度，不猜測前後左右。
這是資料整合，不是人物與雷達的幾何關聯，也不會把未知距離視為空曠。
"""
import copy
import math
import struct
import threading
import time

DIRECTIONS = ("front", "rear", "left", "right")
MODE_NAMES = {0: "latched", 1: "armed_neutral", 2: "running"}
REASONS = (
    "uart_unseen", "uart_stale", "uart_range", "tof_unseen", "tof_status",
    "tof_stale", "obstacle", "wait_clear", "wait_neutral", "wait_new_throttle",
)
FRAME_SIZE = 79
PAYLOAD_SIZE = 72
MAGIC = b"\xa5\x5a"
PERSON_CLASS_NAMES = ("person", "persons", "person(s)", "people", "human", "人", "人物", "行人")


def crc8(data):
    """與 M4 相同：poly=0x07、init=0、不反射、xorout=0。"""
    result = 0
    for value in data:
        result ^= value
        for _ in range(8):
            result = (((result << 1) ^ 0x07) if result & 0x80 else result << 1) & 0xff
    return result


class TelemetryParser:
    """串流解析：可處理拆包、黏包、雜訊及損壞封包；最多保留一個未完成封包。"""

    def __init__(self):
        self._buffer = bytearray()
        self.frames = 0
        self.crc_errors = 0
        self.format_errors = 0
        self.discarded_bytes = 0

    def reset(self):
        self._buffer.clear()

    def feed(self, data):
        self._buffer.extend(data)
        result = []
        while True:
            offset = self._buffer.find(MAGIC)
            if offset < 0:
                keep = 1 if self._buffer[-1:] == MAGIC[:1] else 0
                count = len(self._buffer) - keep
                self.discarded_bytes += count
                del self._buffer[:count]
                break
            if offset:
                self.discarded_bytes += offset
                del self._buffer[:offset]
            if len(self._buffer) < 6:
                break
            if self._buffer[2:5] != b"\x01\x10\x48":
                self.format_errors += 1
                self.discarded_bytes += 1
                del self._buffer[0]
                continue
            if len(self._buffer) < FRAME_SIZE:
                break
            frame = bytes(self._buffer[:FRAME_SIZE])
            if crc8(frame[:-1]) != frame[-1]:
                self.crc_errors += 1
                self.discarded_bytes += 1
                del self._buffer[0]
                continue
            try:
                value = self._decode(frame)
            except ValueError:
                self.format_errors += 1
                del self._buffer[:FRAME_SIZE]
                continue
            del self._buffer[:FRAME_SIZE]
            self.frames += 1
            result.append(value)
        return result

    @staticmethod
    def _decode(frame):
        p = frame[6:-1]
        if p[4] not in MODE_NAMES or p[14] > 1 or p[22] > 1 or p[23] != 0:
            raise ValueError("invalid telemetry state")
        if any(p[n] & 0xf0 for n in (15, 16, 17, 18, 19)):
            raise ValueError("invalid sensor bit mask")
        sensors = []
        for i, direction in enumerate(DIRECTIONS):
            mm, status, seen, age, error, reserved = struct.unpack_from("<HBBHbB", p, 24 + 8 * i)
            if seen > 1 or reserved != 0:
                raise ValueError("invalid sensor record")
            sensors.append(dict(direction=direction, mm=mm, status=status,
                                seen=bool(seen), age_ms=age, api_error=error))
        good, crc_errors, hw_errors, skipped = struct.unpack_from("<IIII", p, 56)
        return {
            "sequence": frame[5], "mcu_uptime_ms": struct.unpack_from("<I", p)[0],
            "mode": p[4], "output_esc": p[5], "output_servo": p[6],
            "command_sequence": p[7], "reasons": struct.unpack_from("<I", p, 8)[0],
            "command_age_ms": struct.unpack_from("<H", p, 12)[0],
            "command_seen": bool(p[14]), "init_failed_mask": p[15],
            "unseen_mask": p[16], "status_mask": p[17], "stale_mask": p[18],
            "obstacle_mask": p[19], "clear_elapsed_ms": struct.unpack_from("<H", p, 20)[0],
            "neutral_seen": bool(p[22]), "sensors": sensors,
            "uart_good_packets": good, "uart_crc_errors": crc_errors,
            "uart_hw_errors": hw_errors, "tx_skipped": skipped,
        }


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _age_ms(now, then):
    return None if then is None else max(0.0, (now - then) * 1000.0)


class SensorStore:
    """一個行程內的執行緒安全資料中心。子行程透過佇列交付相機資料。

    時間使用單調時鐘，M4 年齡使用 MCU 回報的相對時間，不直接比較兩台時鐘。
    雷達與影像只有主機軟體時間，未宣稱感測器曝光／掃描的硬體同步。
    """

    def __init__(self, config=None, clock=time.monotonic, wall_clock=time.time):
        self.config = copy.deepcopy(config or {})
        self._clock, self._wall_clock = clock, wall_clock
        self._lock = threading.RLock()
        self._m4 = None
        self._m4_at = None
        self._m4_reboots = 0
        self._m4_duplicates = 0
        self._lidar = {}
        self._lidar_at = None
        self._lidar_packets = 0
        self._lidar_invalid = 0
        self._camera = None
        self._camera_at = None
        self._camera_received_at = None
        self._person_count = None
        self._person_event_active = False
        self._person_event_count = 0
        self._person_last_event_at = None
        self._person_clear_since = None
        self._person_last_fresh_at = None
        self._errors = {}
        self._validate_config()
        perception = self.config.get("perception", {})
        self._person_class_names = frozenset(
            name.strip().casefold()
            for name in perception.get("person_class_names", PERSON_CLASS_NAMES)
        )
        self._person_min_confidence = float(perception.get("min_confidence", 0.5))
        self._person_clear_hold_s = float(perception.get("clear_hold_s", 1.0))

    def _validate_config(self):
        for section, key, default in (
            ("tof", "stale_ms", 120), ("tof", "telemetry_stale_ms", 250),
            ("lidar", "stale_ms", 500), ("camera", "stale_ms", 1000),
        ):
            value = self.config.get(section, {}).get(key, default)
            if not _number(value) or value <= 0:
                raise ValueError("%s.%s must be positive" % (section, key))
        cfg = self.config.get("lidar", {})
        step = cfg.get("bin_deg", 1)
        if not _number(step) or not 1 <= step <= 45 or 360 % step != 0:
            raise ValueError("lidar.bin_deg must divide 360 and be in [1,45]")
        yaw = cfg.get("yaw_deg")
        if yaw is not None and not _number(yaw):
            raise ValueError("lidar.yaw_deg must be null or a finite angle")
        if not isinstance(cfg.get("clockwise", True), bool):
            raise ValueError("lidar.clockwise must be boolean")
        pos = cfg.get("position_mm", {})
        if not isinstance(pos, dict) or any(not _number(pos.get(k, 0)) for k in ("x", "y", "z")):
            raise ValueError("lidar.position_mm must contain finite coordinates")
        quality = cfg.get("min_quality", 1)
        if not _number(quality) or not 0 <= quality <= 255:
            raise ValueError("lidar.min_quality must be in [0,255]")
        allowance = self.config.get("tof", {}).get("transport_allowance_ms", 10)
        if not _number(allowance) or allowance < 0:
            raise ValueError("tof.transport_allowance_ms must be nonnegative")
        maximum = self.config.get("camera", {}).get("max_detections", 100)
        if not isinstance(maximum, int) or isinstance(maximum, bool) or not 1 <= maximum <= 1000:
            raise ValueError("camera.max_detections must be in [1,1000]")
        perception = self.config.get("perception", {})
        if not isinstance(perception, dict):
            raise ValueError("perception must be an object")
        names = perception.get("person_class_names", list(PERSON_CLASS_NAMES))
        if (not isinstance(names, list) or not names or
                any(not isinstance(name, str) or not name.strip() for name in names)):
            raise ValueError("perception.person_class_names must be a nonempty list of names")
        confidence = perception.get("min_confidence", 0.5)
        if not _number(confidence) or not 0 <= confidence <= 1:
            raise ValueError("perception.min_confidence must be in [0,1]")
        clear_hold = perception.get("clear_hold_s", 1.0)
        if not _number(clear_hold) or clear_hold <= 0:
            raise ValueError("perception.clear_hold_s must be positive")

    def _when(self, value):
        result = self._clock() if value is None else value
        if not _number(result) or result < 0:
            raise ValueError("invalid monotonic timestamp")
        return float(result)

    def set_source_error(self, source, message, now=None):
        if source not in ("m4", "lidar", "camera", "control_network", "lidar_forward"):
            raise ValueError("unknown source")
        with self._lock:
            if message:
                self._errors[source] = {"message": str(message)[:500], "monotonic_s": self._when(now)}
                if source == "camera":
                    # 影像未知不能當成持續無人，也不能據此重新啟用事件。
                    self._person_clear_since = None
                    self._person_last_fresh_at = None
            else:
                self._errors.pop(source, None)

    def update_m4(self, value, received_at=None):
        now = self._when(received_at)
        with self._lock:
            if self._m4_at is not None and now < self._m4_at:
                return False
            if self._m4 is not None:
                old, new = self._m4, value
                if (old["mcu_uptime_ms"], old["sequence"]) == (new["mcu_uptime_ms"], new["sequence"]):
                    self._m4_duplicates += 1
                    return False  # 重複封包不能使舊量測重新變新。
                delta = (new["mcu_uptime_ms"] - old["mcu_uptime_ms"]) & 0xffffffff
                if delta > 0x7fffffff:
                    self._m4_reboots += 1
            self._m4, self._m4_at = copy.deepcopy(value), now
            self._errors.pop("m4", None)
        return True

    def update_lidar(self, angle_deg, distance_mm, quality, received_at=None):
        now = self._when(received_at)
        if not all(_number(x) for x in (angle_deg, distance_mm, quality)):
            return False
        if not 0 <= angle_deg <= 360 or not 0 <= quality <= 255 or quality != int(quality):
            return False
        cfg = self.config.get("lidar", {})
        angle = float(angle_deg) % 360
        slot = int(angle // cfg.get("bin_deg", 1))
        with self._lock:
            if self._lidar_at is not None and now < self._lidar_at:
                return False
            self._lidar_at = now
            self._lidar_packets += 1
            self._errors.pop("lidar", None)
            if distance_mm <= 0 or quality < cfg.get("min_quality", 1):
                self._lidar_invalid += 1
                self._lidar.pop(slot, None)
                return False
            self._lidar[slot] = (angle, float(distance_mm), int(quality), now)
        return True

    def update_camera(self, detections, frame_size, captured_at=None, received_at=None):
        received = self._when(received_at)
        captured = received if captured_at is None else self._when(captured_at)
        if captured > received + 0.01:
            raise ValueError("camera capture timestamp is in the future")
        if len(frame_size) != 2 or any(not _number(v) or v <= 0 for v in frame_size):
            raise ValueError("frame_size must be [width,height]")
        width, height = [int(v) for v in frame_size]
        objects = []
        for item in list(detections)[:self.config.get("camera", {}).get("max_detections", 100)]:
            box, confidence = item.get("bbox_xyxy", []), item.get("confidence")
            cid = item.get("class_id")
            if (len(box) != 4 or not all(_number(v) for v in box)
                    or not _number(confidence) or not 0 <= confidence <= 1
                    or not isinstance(cid, int) or isinstance(cid, bool) or cid < 0):
                continue
            x1, y1, x2, y2 = box
            if x2 < x1 or y2 < y1:
                continue
            track = item.get("track_id")
            if not isinstance(track, int) or isinstance(track, bool) or track < 0:
                track = None
            normalized = {
                "class_id": cid, "class_name": str(item.get("class_name", cid))[:100],
                "confidence": float(confidence), "track_id": track,
                "bbox_xyxy": [max(0.0, min(float(x1), width)), max(0.0, min(float(y1), height)),
                              max(0.0, min(float(x2), width)), max(0.0, min(float(y2), height))],
                "distance_mm": None,  # 尚未建立相機／雷達物件對應，不能捏造人物距離。
            }
            polygon = item.get("polygon")
            if isinstance(polygon, list) and 3 <= len(polygon) <= 128:
                if all(isinstance(p, (list, tuple)) and len(p) == 2 and
                       all(_number(v) for v in p) for p in polygon):
                    normalized["polygon"] = [
                        [max(0.0, min(float(p[0]), width)), max(0.0, min(float(p[1]), height))]
                        for p in polygon
                    ]
                    normalized["polygon_approximate"] = True
            objects.append(normalized)
        with self._lock:
            if self._camera_at is not None and captured <= self._camera_at:
                return False
            self._update_perception(objects, captured, received)
            self._camera = {"frame_size": [width, height], "detections": objects}
            self._camera_at, self._camera_received_at = captured, received
            self._errors.pop("camera", None)
        return True

    def _update_perception(self, objects, captured, received):
        """每份新影像只計算一次人物摘要；呼叫端持有資料鎖。"""
        ttl_s = self.config.get("camera", {}).get("stale_ms", 1000) / 1000.0
        self._person_count = sum(
            item["class_name"].strip().casefold() in self._person_class_names
            and item["confidence"] >= self._person_min_confidence
            for item in objects
        )
        fresh = received - captured <= ttl_s
        continuous = (self._person_last_fresh_at is not None
                      and received - self._person_last_fresh_at <= ttl_s
                      and "camera" not in self._errors)
        if not fresh or not continuous:
            self._person_clear_since = None
        if not fresh:
            self._person_last_fresh_at = None
            return
        self._person_last_fresh_at = captured
        if self._person_count:
            self._person_clear_since = None
            if not self._person_event_active:
                self._person_event_count += 1
                self._person_last_event_at = captured
                self._person_event_active = True
        elif self._person_event_active:
            if self._person_clear_since is None:
                self._person_clear_since = captured
            elif captured - self._person_clear_since >= self._person_clear_hold_s:
                self._person_event_active = False
                self._person_clear_since = None

    def control_health(self, now=None):
        """控制迴圈只取必要健康狀態，不以 90 Hz 複製影像輪廓與雷達點雲。"""
        with self._lock:
            # 先取得鎖再讀時鐘，把等鎖耗時納入資料有效期。
            stamp = self._when(now)
            m4, tof = self._snapshot_m4(stamp)
            return {"m4": m4, "tof": tof, "perception": self._snapshot_perception(stamp)}

    def snapshot(self, now=None):
        with self._lock:
            stamp = self._when(now)
            m4, tof = self._snapshot_m4(stamp)
            lidar = self._snapshot_lidar(stamp)
            camera = self._snapshot_camera(stamp)
            return {
                "schema_version": 1, "timestamp_unix": self._wall_clock(), "monotonic_s": stamp,
                "frame": {"origin": "vehicle_rear", "x": "forward", "y": "right", "z": "up",
                          "distance_unit": "mm", "image_unit": "pixel"},
                "m4": m4, "tof": tof, "lidar": lidar, "camera": camera,
                "perception": self._snapshot_perception(stamp),
                "errors": copy.deepcopy(self._errors),
            }

    def _snapshot_m4(self, now):
        cfg = self.config.get("tof", {})
        rx_age = _age_ms(now, self._m4_at)
        connected = (rx_age is not None and rx_age <= cfg.get("telemetry_stale_ms", 250)
                     and "m4" not in self._errors)
        telemetry = self._m4 or {}
        reasons = telemetry.get("reasons", 0)
        mode = telemetry.get("mode")
        m4 = {k: copy.deepcopy(v) for k, v in telemetry.items() if k not in ("sensors", "reasons")}
        m4.update(connected=connected, age_ms=rx_age, mode=mode,
                  mode_name=MODE_NAMES.get(mode, "unknown"), reasons_mask=reasons,
                  reasons=[s for i, s in enumerate(REASONS) if reasons & (1 << i)],
                  reboot_count=self._m4_reboots, duplicate_packets=self._m4_duplicates,
                  state_current=connected)
        if telemetry.get("command_seen") and rx_age is not None:
            m4["command_age_ms"] = telemetry["command_age_ms"] + rx_age + cfg.get("transport_allowance_ms", 10)
        else:
            m4["command_age_ms"] = None
        tof = {}
        for index, direction in enumerate(DIRECTIONS):
            s = telemetry.get("sensors", [{}] * 4)[index]
            seen = s.get("seen", False)
            age = (s.get("age_ms", 65535) + rx_age + cfg.get("transport_allowance_ms", 10)
                   if seen and rx_age is not None else None)
            if not connected:
                reason = "telemetry_unavailable"
            elif not seen:
                reason = "unseen"
            elif telemetry.get("init_failed_mask", 0) & (1 << index):
                reason = "initialization_failed"
            elif s.get("status", 255) != 0 or s.get("api_error", 0) != 0:
                reason = "range_invalid"
            elif age > cfg.get("stale_ms", 120):
                reason = "stale"
            else:
                reason = "ok"
            valid = reason == "ok"
            # M4 封包的 uptime 與原始 age 使用同一 evaluated_ms：相減可還原
            # 真正的 ToF 讀取時刻。不要用已加主機等待時間的 age 當樣本識別。
            uptime, raw_age = telemetry.get("mcu_uptime_ms"), s.get("age_ms")
            sample_uptime = None
            if (valid and isinstance(uptime, int) and not isinstance(uptime, bool)
                    and 0 <= uptime <= 0xffffffff and isinstance(raw_age, int)
                    and not isinstance(raw_age, bool) and 0 <= raw_age < 65535):
                sample_uptime = (uptime - raw_age) & 0xffffffff
            tof[direction] = {
                "distance_mm": s.get("mm") if valid else None,
                "raw_distance_mm": s.get("mm"), "status": s.get("status", 255),
                "api_error": s.get("api_error"), "seen": seen, "age_ms": age,
                "valid": valid, "reason": reason, "sample_uptime_ms": sample_uptime,
                "guard_obstacle": bool(telemetry.get("obstacle_mask", 0) & (1 << index)) if connected else None,
            }
        return m4, tof

    def _snapshot_lidar(self, now):
        cfg = self.config.get("lidar", {})
        age = _age_ms(now, self._lidar_at)
        ttl, yaw = cfg.get("stale_ms", 500), cfg.get("yaw_deg")
        connected = age is not None and age <= ttl and "lidar" not in self._errors
        position = cfg.get("position_mm", {})
        step = cfg.get("bin_deg", 1)
        sign = 1 if cfg.get("clockwise", True) else -1
        points = []
        sectors = {d: {"nearest_return_mm": None, "valid_bins": 0, "coverage_ratio": 0.0}
                   for d in DIRECTIONS} if yaw is not None else None
        for slot, (angle, distance, quality, when) in sorted(self._lidar.items()):
            point_age = _age_ms(now, when)
            if not connected or point_age > ttl:
                continue
            p = {"raw_angle_deg": angle, "distance_mm": distance,
                 "quality": quality, "age_ms": point_age}
            if yaw is not None:
                heading = (yaw + sign * angle) % 360
                radians = math.radians(heading)
                p.update(vehicle_angle_deg=heading,
                         vehicle_x_mm=position.get("x", 0) + distance * math.cos(radians),
                         vehicle_y_mm=position.get("y", 0) + distance * math.sin(radians),
                         vehicle_z_mm=position.get("z", 0))
                sector = ("front", "right", "rear", "left")[int((heading + 45) % 360 // 90)]
                target = sectors[sector]
                target["valid_bins"] += 1
                target["nearest_return_mm"] = (distance if target["nearest_return_mm"] is None
                                                else min(distance, target["nearest_return_mm"]))
            points.append(p)
        if sectors is not None:
            for value in sectors.values():
                value["coverage_ratio"] = min(1.0, value["valid_bins"] * step / 90.0)
        return {
            "connected": connected, "valid": connected and bool(points), "age_ms": age,
            "orientation_configured": yaw is not None, "yaw_deg": yaw,
            "position_mm": copy.deepcopy(position), "bin_deg": step, "points": points,
            "sectors": sectors, "packets": self._lidar_packets, "invalid_returns": self._lidar_invalid,
            "timing_basis": "jetson_receive_monotonic", "coverage_is_free_space": False,
        }

    def _camera_health(self, now):
        age = _age_ms(now, self._camera_at)
        valid = (age is not None and age <= self.config.get("camera", {}).get("stale_ms", 1000)
                 and "camera" not in self._errors)
        return age, valid

    def _snapshot_perception(self, now):
        age, valid = self._camera_health(now)
        return {
            "valid": valid, "age_ms": age, "captured_at": self._camera_at,
            "person_count": self._person_count if valid else None,
            "person_present": bool(self._person_count) if valid else None,
            "event_count": self._person_event_count,
            "last_event_at": self._person_last_event_at,
        }

    def _snapshot_camera(self, now):
        age, valid = self._camera_health(now)
        return {
            "valid": valid, "age_ms": age,
            "received_age_ms": _age_ms(now, self._camera_received_at),
            "processing_delay_ms": (None if self._camera_at is None else
                                    max(0, (self._camera_received_at - self._camera_at) * 1000)),
            "frame_size": copy.deepcopy(self._camera["frame_size"]) if self._camera else None,
            "detections": copy.deepcopy(self._camera["detections"]) if valid else [],
            "timing_basis": "jetson_capture_monotonic", "object_range_association": False,
        }
