"""雷達實體車位停車；隔離程序只產生有期限的建議，不碰 UART。

車位內部座標為後軸中心、X 前、Y 左、逆時針角度；所有長度以公尺。
每次行駛都需要新的牆面定位與最新掃描的路徑檢查，不按馬達時間推算位置。
"""
import math
import queue
import time

from .parking_config import validate_settings


def wrap_angle(value):
    return (value + math.pi) % (2 * math.pi) - math.pi


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def put_latest(channel, value):
    try:
        channel.put_nowait(value)
        return
    except queue.Full:
        pass
    try:
        channel.get_nowait()
    except queue.Empty:
        pass
    try:
        channel.put_nowait(value)
    except queue.Full:
        pass


def extract_scan(snapshot, cfg, now):
    """只使用帶接收年齡的真實回波；返回點、各角度來源時刻與最舊來源時間。

    不把收到某一新封包當成整圈掃描更新；以每個角度桶的時刻辨認新資料。
    SDK 沒有硬體掃描時戳，因此此處仍是 Jetson 接收時間，非硬體同步。
    """
    import numpy as np
    lidar = snapshot.get("lidar", {})
    stamp = snapshot.get("monotonic_s")
    if (not finite(stamp) or stamp < 0 or stamp > now or now - stamp > cfg["proposal_timeout_s"]
            or lidar.get("connected") is not True or lidar.get("valid") is not True):
        return None
    points, times, bins = [], [], {}
    origin = cfg["lidar_origin_body_m"]
    sign = 1 if cfg["lidar_clockwise"] else -1
    width, rear = cfg["body_width_m"], cfg["rear_overhang_m"]
    front = cfg["body_length_m"] - rear
    raw_points = lidar.get("points", [])
    if not isinstance(raw_points, list) or len(raw_points) > 1000:
        return None
    for p in raw_points:
        if not isinstance(p, dict):
            continue
        angle, distance, age, quality = (p.get(k) for k in ("raw_angle_deg", "distance_mm", "age_ms", "quality"))
        if not all(finite(v) for v in (angle, distance, age, quality)):
            continue
        when = stamp - age / 1000.0
        if not (0 <= angle < 360 and 0.08 <= distance / 1000 <= cfg["scan_range_m"]
                and quality >= 1 and age >= 0 and 0 <= when <= now
                and now - when <= cfg["max_scan_age_s"]):
            continue
        theta = math.radians(cfg["lidar_yaw_deg"] + sign * angle)
        x = origin[0] + distance / 1000.0 * math.cos(theta)
        y = origin[1] - distance / 1000.0 * math.sin(theta)
        # 車體自己的回波不成為車位牆；刪除該射線，絕不向其後方填自由空間。
        if -rear <= x <= front and abs(y) <= width / 2:
            continue
        key = int(angle)
        previous = bins.get(key)
        if previous is None or when > previous[0]:
            bins[key] = (when, x, y)
    if len(bins) < cfg["min_scan_points"]:
        return None
    for key, (when, x, y) in sorted(bins.items()):
        points.append((x, y))
        times.append(when)
    tokens = {key: item[0] for key, item in bins.items()}
    return {"points": np.asarray(points, dtype=float), "tokens": tokens,
            "source_at": min(times), "observation_at": float(np.median(times))}


def curvature_to_servo(curvature, cfg):
    """左右校正點間以曲率插值；倒車仍使用相同實體轉向定義。"""
    limit = 1.0 / cfg["min_turn_radius_m"]
    curvature = max(-limit, min(limit, float(curvature)))
    if curvature >= 0:
        fraction = min(1.0, curvature * cfg["left_turn_radius_m"])
        target = cfg["steering_left_pwm"]
    else:
        fraction = min(1.0, -curvature * cfg["right_turn_radius_m"])
        target = cfg["steering_right_pwm"]
    return int(round(152 + fraction * (target - 152)))


class AutomaticParkingEngine:
    """以獨立程序呼叫 step；新 session/epoch 重新建立任務。"""
    def __init__(self, config, clock=time.monotonic):
        self.cfg = validate_settings(config.get("auto_parking", config), require_enabled=True)
        self.clock = clock
        self.key = None
        self.state = "idle"
        self.reset()

    def reset(self):
        self.key = None
        self.state = "idle"
        self.reason = "park_ready"
        self.pose = None
        self.last_pose = None
        self.last_observation_at = None
        self.tokens = {}
        self.last_source_at = None
        self.progress = 0.0
        self.path_remaining = None
        self.fit_error = None
        self.started = self.state_at = self.progress_at = None
        self.best_progress = 0.0
        self.base_progress = 0.0
        self.segments = []
        self.segment_index = 0
        self.reboot = None
        self._clear_stable()

    @property
    def target(self):
        return [self.cfg["target_back_clearance_m"] + self.cfg["rear_overhang_m"], 0.0, 0.0]

    def _clear_stable(self):
        self.stable_start = None
        self.stable_pose = None
        self.stable_count = 0

    def _neutral(self, snapshot):
        m4 = snapshot.get("m4", {})
        return m4.get("output_esc") == 152 and m4.get("output_servo") == 152

    def _stable(self, snapshot, stamp, duration):
        if not self._neutral(snapshot):
            self._clear_stable()
            return False
        if (self.stable_pose is None
                or math.hypot(self.pose[0] - self.stable_pose[0], self.pose[1] - self.stable_pose[1]) > .025
                or abs(wrap_angle(self.pose[2] - self.stable_pose[2])) > math.radians(2)):
            self.stable_start = stamp
            self.stable_pose = list(self.pose)
            self.stable_count = 1
        else:
            self.stable_count += 1
        return self.stable_count >= self.cfg["min_stable_samples"] and stamp - self.stable_start >= duration

    def _report(self, message, reason=None, esc=152, servo=152, stop=False):
        created = self.clock()
        at = message["at"]
        source = self.last_source_at if self.last_source_at is not None else at
        report = {"session": self.key[0], "epoch": self.key[1], "input_at": at,
                "created_at": created, "source_at": source, "esc": int(esc), "servo": int(servo),
                "stop": bool(stop), "state": self.state, "reason": reason or "park_" + self.state,
                "completed": self.state == "complete", "range_source": "lidar_tof",
                "pose": list(self.pose) if self.pose is not None else None,
                "target": self.target, "progress_m": self.base_progress + self.progress}
        if self.path_remaining is not None:
            report["path_remaining_m"] = self.path_remaining
        if self.fit_error is not None:
            report["fit_error_m"] = self.fit_error
        return report

    def _abort(self, message, reason):
        self.state, self.reason = "aborted", reason
        return self._report(message, reason, stop=True)

    def _sensor_reason(self, snapshot):
        m4 = snapshot.get("m4", {})
        if m4.get("connected") is not True or m4.get("mode") not in (1, 2):
            return "park_sensor_invalid"
        reboot = m4.get("reboot_count")
        if not isinstance(reboot, int) or isinstance(reboot, bool) or reboot < 0:
            return "park_sensor_invalid"
        if self.reboot is None:
            self.reboot = reboot
        elif self.reboot != reboot:
            return "park_m4_reboot"
        for direction in ("front", "rear", "left", "right"):
            v = snapshot.get("tof", {}).get(direction, {})
            d = v.get("distance_mm")
            if v.get("valid") is not True or not finite(d):
                return "park_sensor_invalid"
            if d <= self.cfg["tof_clearance_mm"]:
                return "park_path_blocked"
        return None

    def _new_scan(self, scan):
        count = sum(1 for key, stamp in scan["tokens"].items()
                    if stamp > self.tokens.get(key, -1) + 1e-6)
        if count < self.cfg["min_new_points"]:
            return False
        if self.last_source_at is not None and scan["source_at"] <= self.last_source_at + 1e-6:
            return False
        self.tokens = dict(scan["tokens"])
        self.last_source_at = scan["source_at"]
        return True

    def _accept_pose(self, candidate, observation_at):
        pose = candidate.get("pose", [])
        if len(pose) != 3 or not all(finite(x) for x in pose):
            return "park_pose_lost"
        pose = list(map(float, pose))
        if self.last_pose is not None:
            dt = observation_at - self.last_observation_at
            if dt <= 0 or dt > .6:
                return "park_pose_lost"
            delta = math.hypot(pose[0] - self.last_pose[0], pose[1] - self.last_pose[1])
            angle = abs(wrap_angle(pose[2] - self.last_pose[2]))
            if delta > self.cfg["max_speed_m_s"] * dt + .035:
                return "park_speed_exceeded"
            if angle > self.cfg["max_yaw_rate_rad_s"] * dt + math.radians(3):
                return "park_pose_jump"
        self.pose, self.last_pose = pose, list(pose)
        self.last_observation_at = observation_at
        self.fit_error = float(candidate.get("fit_error_m", 0.0))
        return None

    def _at_goal(self):
        return (math.hypot(self.pose[0] - self.target[0], self.pose[1]) <= self.cfg["position_tolerance_m"]
                and abs(wrap_angle(self.pose[2])) <= math.radians(self.cfg["yaw_tolerance_deg"]))

    def step(self, message):
        from .parking_geometry import detect_bays, track_bay
        from .parking_path import plan_path, split_path, follow_path, check_motion
        if not isinstance(message, dict):
            return None
        control = message.get("control", {})
        epoch, session, at = control.get("epoch"), message.get("session"), message.get("at")
        now = self.clock()
        if (not isinstance(session, str) or not isinstance(epoch, int) or isinstance(epoch, bool)
                or not finite(at) or not 0 <= at <= now):
            return None
        if control.get("mode") != "park":
            self.reset()
            return None
        key = (session, epoch)
        if self.key != key:
            self.reset()
            self.key = key
            self.state = "searching"
            self.started = self.state_at = now
            # 巡邏確認的車位是本次唯一目標，不能改選目前最近的另一個車位。
            # 新鮮掃描仍須重新追蹤、停穩與規劃，不使用通知時的舊路徑。
            mission = control.get("mission", {})
            if isinstance(mission, dict) and mission.get("state") == "parking":
                approved = mission.get("approved_pose")
                if (not isinstance(approved, (list, tuple)) or len(approved) != 3
                        or not all(finite(value) for value in approved)):
                    return self._abort(message, "park_approved_target_missing")
                self.pose = list(map(float, approved))
                self.state = "acquiring"
        if self.state in ("aborted", "complete"):
            return self._report(message, self.reason, stop=True)
        if now - at > self.cfg["proposal_timeout_s"]:
            # 計畫完成後丟棄排隊的舊掃描；父端仍維持中立且有固定規劃期限。
            if self.state in ("searching", "acquiring", "planning", "gear_wait") and self.base_progress == 0:
                return None
            return self._abort(message, "park_worker_stale")
        if now - self.started >= self.cfg["mission_timeout_s"]:
            return self._abort(message, "park_timeout")
        if self.state in ("searching", "acquiring") and now - self.started >= self.cfg["search_timeout_s"]:
            return self._abort(message, "park_no_bay")
        snapshot = message.get("snapshot", {})
        problem = self._sensor_reason(snapshot)
        if problem:
            return self._abort(message, problem)
        scan = extract_scan(snapshot, self.cfg, now)
        if scan is None:
            return self._abort(message, "park_lidar_unavailable")
        if not self._new_scan(scan):
            return None
        points, observed = scan["points"], scan["observation_at"]
        if self.pose is None:
            candidates = detect_bays(points, self.cfg)
            if not candidates:
                return self._report(message, "park_searching")
            # 多個已明確辨識的車位按入口距離選最近者；選定後不可換位。
            candidates.sort(key=lambda c: (math.hypot(c["pose"][0] - self.cfg["bay_depth_m"], c["pose"][1]),
                                           c["pose"][0], c["pose"][1]))
            candidate = candidates[0]
            self.state, self.state_at = "acquiring", now
        else:
            candidate = track_bay(points, self.cfg, self.pose)
            if candidate is None:
                return self._abort(message, "park_pose_lost")
        # 規劃期間沒有運動命令；新掃描必須仍接近原規劃位置，才可重新建立dt基準。
        if self.state == "gear_wait" and self.last_observation_at is not None and observed-self.last_observation_at > .6:
            old = self.last_pose
            if (math.hypot(candidate["pose"][0]-old[0], candidate["pose"][1]-old[1]) > .04
                    or abs(wrap_angle(candidate["pose"][2]-old[2])) > math.radians(3)):
                return self._abort(message, "park_pose_jump")
            self.last_pose = self.last_observation_at = None
        problem = self._accept_pose(candidate, observed)
        if problem:
            return self._abort(message, problem)
        if self.state == "acquiring":
            if self._stable(snapshot, observed, self.cfg["prepare_s"]):
                self.state, self.state_at = "planning", now
                self._clear_stable()
            return self._report(message)
        if self.state == "planning":
            if not self._neutral(snapshot):
                return self._abort(message, "park_prepare_timeout")
            planned = plan_path(self.pose, self.target, points, self.cfg)
            if not planned.get("ok"):
                return self._abort(message, "park_no_path")
            self.segments = split_path(planned["path"])
            if not self.segments or self.segments[-1][-1][3] != -1:
                return self._abort(message, "park_no_path")
            self.segment_index = 0
            self.state, self.state_at = "gear_wait", self.clock()
            self._clear_stable()
            # 禁止把數秒前計畫輸入包裝成新鮮油門；必須等下一筆掃描。
            return None
        if self.state == "gear_wait":
            if now - self.state_at >= self.cfg["gear_timeout_s"]:
                return self._abort(message, "park_gear_timeout")
            if not self._stable(snapshot, observed, self.cfg["gear_wait_s"]):
                return self._report(message)
            gear = self.segments[self.segment_index][-1][3]
            self.state = "aligning" if gear > 0 else "reversing"
            self.state_at = self.progress_at = now
            self.best_progress = self.progress
            self._clear_stable()
        if self.state in ("aligning", "reversing"):
            # 最後倒車段已達真正車位位置／角度門檻時，立即中立並觀察穩定。
            # 不為追逐離散路徑更小的數值誤差而繼續越過已合格的停車位置。
            if (self.state == "reversing" and self.segment_index == len(self.segments) - 1
                    and self._at_goal()):
                self.state, self.state_at = "settling", now
                self._clear_stable()
                return self._report(message)
            if now - self.progress_at >= self.cfg["no_progress_s"]:
                return self._abort(message, "park_no_progress")
            segment = self.segments[self.segment_index]
            result = follow_path(segment, self.pose, self.progress, self.cfg["lookahead_m"], self.cfg)
            if not result.get("ok") or result.get("tracking_error_m", float("inf")) > self.cfg["max_cross_track_m"]:
                return self._abort(message, "park_tracking_error")
            self.progress = float(result["progress_m"])
            self.path_remaining = float(result["remaining_m"])
            if self.progress >= self.best_progress + self.cfg["min_progress_m"]:
                self.best_progress, self.progress_at = self.progress, now
            if result.get("completed"):
                if self.segment_index == len(self.segments) - 1:
                    if not self._at_goal():
                        return self._abort(message, "park_target_out_of_range")
                    self.state, self.state_at = "settling", now
                else:
                    self.segment_index += 1
                    self.base_progress += self.progress
                    self.progress = self.best_progress = 0.0
                    self.state, self.state_at = "gear_wait", now
                self._clear_stable()
                return self._report(message)
            gear = int(result["gear"])
            curvature = float(result["signed_curvature"])
            check = check_motion(self.pose, points, self.cfg, gear, curvature, .18)
            clear = check if isinstance(check, bool) else check.get("ok") is True
            if not clear:
                return self._abort(message, "park_path_blocked")
            return self._report(message, esc=self.cfg["forward_esc"] if gear > 0 else self.cfg["reverse_esc"],
                                servo=curvature_to_servo(curvature, self.cfg))
        if self.state == "settling":
            if now - self.state_at >= self.cfg["settle_timeout_s"]:
                return self._abort(message, "park_settle_timeout")
            if not self._at_goal():
                return self._abort(message, "park_target_out_of_range")
            if self._stable(snapshot, observed, self.cfg["settle_s"]):
                self.state, self.reason = "complete", "park_complete"
                return self._report(message, stop=True)
            return self._report(message)
        return self._abort(message, "park_internal_error")


def parking_worker(config, input_queue, output_queue, stop_event):
    """子程序沒有裝置描述元；計算堵塞時主端仍能中立停止。"""
    from .parking_scout import ParkingScout
    engine = AutomaticParkingEngine(config)
    scout = ParkingScout(config)
    try:
        while not stop_event.is_set():
            try:
                message = input_queue.get(timeout=.1)
            except queue.Empty:
                continue
            for _ in range(8):
                try:
                    message = input_queue.get_nowait()
                except queue.Empty:
                    break
            mode = message.get("control", {}).get("mode") if isinstance(message, dict) else None
            if mode in ("search", "offer"):
                engine.reset()
                proposal = scout.step(message)
            else:
                scout.reset()
                proposal = engine.step(message)
            if proposal is not None:
                put_latest(output_queue, proposal)
    finally:
        # 子程序退出不等待主端已停止讀取的佇列。
        output_queue.cancel_join_thread()
