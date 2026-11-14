"""實體通訊工作執行緒；UART 僅由 UartBridge 一處開啟。"""
import json
import math
import select
import socket
import threading
import time

from .core import TelemetryParser
from .runtime import open_exclusive_uart, close_exclusive_uart


def crc8(data):
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ 0x07) & 0xFF if value & 0x80 else (value << 1) & 0xFF
    return value


def build_command(speed, turn, sequence):
    body = bytes((0x55, 0xAA, 0x02, sequence & 0xFF, speed & 0xFF, turn & 0xFF))
    return body + bytes((crc8(body),))


def write_packet(serial_port, packet, timeout_s=0.03):
    """完成同一封包的部分寫入；逾時即報錯，呼叫端不得重送油門。"""
    deadline = time.monotonic() + timeout_s
    offset = 0
    while offset < len(packet):
        if time.monotonic() >= deadline:
            raise TimeoutError("UART 完整封包寫入逾時")
        written = serial_port.write(packet[offset:])
        if written is None or written < 0 or written > len(packet) - offset:
            raise IOError("UART 回報無效寫入長度")
        if not written:
            time.sleep(0.001)
            continue
        offset += written


def latest_control_packet(sock, remote, limit=32):
    """有界清空 UDP 佇列；未清空則丟棄整批，避免舊油門逐筆重播。

    舊版二位元組協定不再接受。只有設定中的電腦位址可更新控制權。
    不把封包中的傳送時間視為可信的量測時間。
    """
    latest = None
    for _ in range(limit):
        try:
            data, address = sock.recvfrom(4097)
        except BlockingIOError:
            return latest
        if address != remote or len(data) > 4096:
            continue
        try:
            packet = json.loads(data.decode("utf-8"))
        except (ValueError, UnicodeError, RecursionError):
            continue
        if isinstance(packet, dict):
            latest = packet
    return None


def status_message(authority, health, kind="status", now=None):
    """同一 UDP 插孔回傳精簡狀態，沿用既有 NAT 映射與 5000 埠。"""
    control = authority.snapshot(now=now)
    m4, tof = health.get("m4", {}), health.get("tof", {})
    blocked = control.get("reason") if control.get("mode") == "fault" else None
    if blocked is None and not m4.get("connected"):
        blocked = "m4_offline"
    if blocked is None:
        blocked = next(("tof_invalid_" + d for d in ("front", "rear", "left", "right")
                        if not tof.get(d, {}).get("valid")), None)
    if blocked is None and m4.get("mode") not in (1, 2):
        blocked = "m4_guard_latched" if m4.get("mode") == 0 else "m4_state_unknown"
    if blocked is None:
        blocked = control.get("video", {}).get("reason")
    return {
        "v": 1, "type": kind, "session": authority.session, "control": control,
        "health": {
            "m4_connected": bool(m4.get("connected")),
            "m4_mode": m4.get("mode_name", "unknown"),
            "tof_mm": {d: tof.get(d, {}).get("distance_mm")
                       for d in ("front", "rear", "left", "right")},
            "blocked_reason": blocked,
        },
    }


class UartBridge(threading.Thread):
    """UDP 控制請求交給唯一控制管理器；UART 由本執行緒固定週期輸出。"""

    def __init__(self, store, config, stop_event, authority, serial_factory=None):
        super().__init__(name="m4_uart_bridge", daemon=True)
        self.store, self.config, self.stop_event = store, config, stop_event
        self.authority = authority
        self.serial_factory = serial_factory
        self.failure = None

    def run(self):
        ser = sock = None
        sequence = 0
        transport_failed = False
        exclusive_uart = self.serial_factory is None
        try:
            uart, network = self.config["uart"], self.config["network"]
            settings = self.config.get("control", {})
            if exclusive_uart:
                ser = open_exclusive_uart(uart["device"], int(uart["baud"]), write_timeout=0.02)
            else:
                # 注入工廠只供無硬體測試；正式執行必須取得串口排他權。
                ser = self.serial_factory(uart["device"], int(uart["baud"]), timeout=0,
                                          write_timeout=0.02, rtscts=False, dsrdtr=False, xonxoff=False)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            remote = (socket.gethostbyname(network["pc_host"]), int(network["control_port"]))
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setblocking(False)
            parser = TelemetryParser()
            next_hello = next_status = next_output = 0.0
            poll_s = min(0.01, max(0.001, float(uart.get("poll_interval_s", 0.005))))
            output_s = float(settings.get("output_interval_s", 0.011))
            status_s = float(settings.get("status_interval_s", 0.1))
            hello_s = float(network.get("hello_interval_s", 2.0))
            if not all(math.isfinite(v) and v > 0 for v in (output_s, status_s, hello_s)):
                raise ValueError("控制輸出、狀態回傳與 HELLO 週期必須為正數")
            print("[M4] {} @ {}；控制端 {}:{}；請使用新版電腦端控制器。".format(
                uart["device"], uart["baud"], *remote), flush=True)
            while not self.stop_event.is_set():
                # 即使電腦尚未啟動或完全沒有 UDP 命令，仍持續接收四向距離。
                received_at = time.monotonic()
                incoming = ser.read(min(4096, max(1, int(ser.in_waiting))))
                if incoming:
                    frames = parser.feed(incoming)
                    # 積壓時先消化輸入，不把舊回報逐筆標成剛收到的新量測。
                    if frames and int(ser.in_waiting) == 0:
                        self.store.update_m4(frames[-1], received_at=received_at)

                now = time.monotonic()
                health = self.store.control_health(now=now)
                try:
                    latest = latest_control_packet(sock, remote)
                except OSError as exc:
                    self.store.set_source_error("control_network", str(exc))
                    latest = None
                if latest is not None:
                    if self.authority.receive(latest, health, now=now):
                        self.store.set_source_error("control_network", None)

                # 單一寫入者；即使沒有遠端封包也輸出待命／故障中立值。
                # 控制權管理器保留遠端心跳逾時，不因持續 UART 中立命令而失效。
                now = time.monotonic()
                if now >= next_output:
                    try:
                        self.authority.dispatch_output(
                            self.store.control_health,
                            lambda esc, servo: write_packet(ser, build_command(esc, servo, sequence)))
                    except Exception:
                        transport_failed = True
                        raise
                    sequence = (sequence + 1) & 0xFF
                    next_output = time.monotonic() + output_s

                now = time.monotonic()
                kind = "hello" if now >= next_hello else "status" if now >= next_status else None
                if kind is not None:
                    health = self.store.control_health(now=now)
                    message = status_message(self.authority, health, kind=kind, now=now)
                    try:
                        sock.sendto(json.dumps(message, ensure_ascii=True, allow_nan=False,
                                               separators=(",", ":")).encode("utf-8"), remote)
                        self.store.set_source_error("control_network", None)
                    except OSError as exc:
                        # 電腦關閉或網路失聯不終止 UART 遙測與中立輸出。
                        self.store.set_source_error("control_network", str(exc))
                    if kind == "hello":
                        next_hello = now + hello_s
                    next_status = now + status_s

                # 不為補足落後週期而重播舊命令；下一輪重新檢查控制權與健康狀態。
                timeout = min(poll_s, max(0.0, next_output - time.monotonic()))
                try:
                    select.select([sock], [], [], timeout)
                except OSError as exc:
                    self.store.set_source_error("control_network", str(exc))
                    self.stop_event.wait(timeout)
        except Exception as exc:
            self.failure = str(exc)
            self.store.set_source_error("m4", self.failure)
            self.authority.fault("m4_transport_error")
            print("[M4] 通訊已停止：{}；底層命令逾時保護接手，修復後重新啟動。".format(exc), flush=True)
            # 占用或裝置錯誤時結束整套新實例，避免留下相機等背景工作。
            self.stop_event.set()
        finally:
            if ser is not None:
                try:
                    if transport_failed:
                        # 有部分寫入錯誤時，不接續舊封包或重播油門。
                        ser.reset_output_buffer()
                    else:
                        write_packet(ser, build_command(152, 152, sequence))
                except Exception:
                    pass
                try:
                    if exclusive_uart:
                        close_exclusive_uart(ser)
                    else:
                        ser.close()
                except Exception:
                    pass
            if sock is not None:
                sock.close()


def parse_lidar_csv(data):
    fields = data.decode("ascii").strip().split(",")
    if len(fields) != 3:
        raise ValueError("雷達封包必須為 angle,distance,quality")
    angle, distance, quality = float(fields[0]), float(fields[1]), int(fields[2])
    if not math.isfinite(angle) or not math.isfinite(distance):
        raise ValueError("雷達角度與距離必須為有限數值")
    if not 0.0 <= angle < 360.0 or distance < 0.0 or not 0 <= quality <= 255:
        raise ValueError("雷達封包數值超出範圍")
    return angle, distance, quality


class LidarReceiver(threading.Thread):
    """保留 SDK 原始 UDP 格式與轉送，另更新同程序的感測資料庫。"""

    def __init__(self, store, config, stop_event):
        super().__init__(name="lidar_receive_forward", daemon=True)
        self.store, self.config, self.stop_event = store, config, stop_event

    def run(self):
        receiver = sender = None
        try:
            network = self.config["network"]
            receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            receiver.bind((network.get("lidar_local_host", "127.0.0.1"), int(network["lidar_local_port"])))
            receiver.settimeout(0.1)
            sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sender.setblocking(False)
            remote = (socket.gethostbyname(network["pc_host"]), int(network["lidar_remote_port"]))
            forwarding_error_reported = False
            while not self.stop_event.is_set():
                try:
                    data, _ = receiver.recvfrom(1024)
                except socket.timeout:
                    continue
                received_at = time.monotonic()
                try:
                    angle, distance, quality = parse_lidar_csv(data)
                    self.store.update_lidar(angle, distance, quality, received_at=received_at)
                except (ValueError, UnicodeError) as exc:
                    self.store.set_source_error("lidar", str(exc))
                    continue
                try:
                    sender.sendto(data, remote)
                    self.store.set_source_error("lidar_forward", None)
                    forwarding_error_reported = False
                except OSError as exc:
                    self.store.set_source_error("lidar_forward", str(exc))
                    if not forwarding_error_reported:
                        print("[雷達] 遠端轉送失敗，車端仍接收：{}".format(exc), flush=True)
                        forwarding_error_reported = True
        except Exception as exc:
            self.store.set_source_error("lidar", str(exc))
            print("[雷達] 接收已停止：{}".format(exc), flush=True)
        finally:
            if receiver is not None:
                receiver.close()
            if sender is not None:
                sender.close()
