#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MiniTesla 第十階段：巡邏找位與隔離自動停車運算，保留唯一 UART 控制寫入者。"""
import argparse
import json
import math
import multiprocessing
import os
from pathlib import Path
import queue
import select
import signal
import socket
import struct
import time
import traceback

from minitesla_sensors.core import SensorStore
from minitesla_sensors.control import ControlAuthority
from minitesla_sensors.io import LidarReceiver, UartBridge
from minitesla_sensors.runtime import SingleInstance, SingleInstanceError


ROOT = Path(__file__).resolve().parent


def read_boot_id():
    try:
        return Path("/proc/sys/kernel/random/boot_id").read_text(encoding="ascii").strip() or None
    except OSError:
        return None


BOOT_ID = read_boot_id()


class ShutdownRequest:
    """訊號處理函式只設旗標，避免中斷 Event.wait 時再次取得同步鎖。"""

    def __init__(self):
        self.signal_number = None

    def request(self, signum, frame=None):
        if self.signal_number is None:
            self.signal_number = signum

    @property
    def requested(self):
        return self.signal_number is not None

    @property
    def reason(self):
        if self.signal_number == signal.SIGTERM:
            return "signal_sigterm"
        if self.signal_number == signal.SIGINT:
            return "signal_sigint"
        return "local_stop"


def put_latest(target_queue, item):
    """佇列有界；忙碌時捨棄舊資料，絕不阻塞影像推論。"""
    try:
        target_queue.put_nowait(item)
        return
    except queue.Full:
        pass
    try:
        target_queue.get_nowait()
    except queue.Empty:
        pass
    try:
        target_queue.put_nowait(item)
    except queue.Full:
        # multiprocessing 的 feeder 尚未送出舊資料時，本次允許掉幀。
        pass


def newest_available(target_queue, first, limit=8):
    latest = first
    for _ in range(limit):
        try:
            latest = target_queue.get_nowait()
        except queue.Empty:
            break
    return latest


def camera_worker(config, metadata_queue, video_queue, display_queue, stop_event):
    cap = None
    try:
        # 延後匯入，讓純感測監看及離線測試不需要 CUDA 或相機套件。
        import cv2
        import numpy as np
        from ultralytics import YOLO

        camera = config["camera"]
        model = YOLO(camera["model"])
        cap = cv2.VideoCapture(camera["index"])
        if not cap.isOpened():
            raise RuntimeError("無法開啟攝影機 {}".format(camera["index"]))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 部分後端不支援；不將其視為擷取延遲保證。
        jpeg_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(camera.get("jpeg_quality", 99))]
        maximum = int(camera.get("max_detections", 100))

        def color(identifier):
            return tuple(int(x) for x in np.random.RandomState(identifier & 0xFFFFFFFF).randint(0, 255, 3))

        while not stop_event.is_set():
            # 在擷取前記時，將擷取與推論耗時納入有效期限。
            captured_at = time.monotonic()
            success, frame = cap.read()
            if not success:
                raise RuntimeError("攝影機未回傳影格")
            height, width = frame.shape[:2]
            results = model.track(source=frame, tracker="bytetrack.yaml", persist=True,
                                  stream=False, conf=float(camera.get("confidence", 0.25)),
                                  iou=float(camera.get("iou", 0.7)), verbose=False)
            detections = []
            for result in results:
                boxes = result.boxes
                if boxes is None:
                    continue
                coordinates = boxes.xyxy.cpu().numpy()
                confidences = boxes.conf.cpu().numpy()
                classes = boxes.cls.cpu().numpy().astype(int)
                identities = boxes.id.cpu().numpy().astype(int) if boxes.id is not None else None
                polygons = result.masks.xy if result.masks is not None else []
                names = result.names
                for index, xyxy in enumerate(coordinates):
                    class_id = int(classes[index])
                    track_id = int(identities[index]) if identities is not None else None
                    confidence = float(confidences[index])
                    class_name = str(names.get(class_id, "class_{}".format(class_id))) if isinstance(names, dict) else str(names[class_id])
                    draw_color = color(track_id if track_id is not None else class_id)
                    detection = {"class_id": class_id, "class_name": class_name,
                                 "confidence": confidence, "track_id": track_id,
                                 "bbox_xyxy": [float(x) for x in xyxy]}
                    if index < len(polygons):
                        polygon = polygons[index]
                        cv2.polylines(frame, [polygon.reshape(-1, 1, 2).astype(int)],
                                      isClosed=True, color=draw_color, thickness=2)
                        # 感測中介資料限制每個輪廓最多 128 點；原顯示仍畫完整輪廓。
                        step = max(1, int(math.ceil(len(polygon) / 128.0)))
                        detection["polygon"] = polygon[::step].astype(float).tolist()
                    if len(detections) < maximum:
                        detections.append(detection)
                    cv2.rectangle(frame, (int(xyxy[0]), int(xyxy[1])),
                                  (int(xyxy[2]), int(xyxy[3])), draw_color, 2)
                    label = "ID {} {} {:.2f}".format(track_id if track_id is not None else -1, class_name, confidence)
                    cv2.putText(frame, label, (int(xyxy[0]), int(xyxy[1]) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw_color, 1)
            put_latest(metadata_queue, {"kind": "camera", "captured_at": captured_at,
                                       "frame_size": [width, height], "detections": detections})
            if display_queue is not None:
                put_latest(display_queue, frame)
            encoded, image = cv2.imencode(".jpg", frame, jpeg_params)
            if encoded:
                # 第四階段：原始 JPEG，傳送程序加上影格編號並等待電腦回覆。
                put_latest(video_queue, (captured_at, image.tobytes()))
    except Exception as exc:
        put_latest(metadata_queue, {"kind": "error", "message": str(exc), "at": time.monotonic()})
        print("[相機] 工作程序已停止：{}".format(exc), flush=True)
    finally:
        if cap is not None:
            cap.release()


VIDEO_HEADER = struct.Struct("!4sQI")
VIDEO_ACK = struct.Struct("!4sQ")
MAX_JPEG_BYTES = 8 * 1024 * 1024


def receive_exact(client, size, deadline, stop_event):
    """同一絕對期限內讀足資料；拆包、半包與 EOF 都有界處理。"""
    parts = bytearray()
    while len(parts) < size:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or stop_event.is_set():
            raise TimeoutError("影像確認逾時或停止")
        client.settimeout(min(0.2, remaining))
        try:
            block = client.recv(size - len(parts))
        except socket.timeout:
            continue
        if not block:
            raise EOFError("影像接收端已關閉")
        parts.extend(block)
    return bytes(parts)


def transmit_frame(client, captured_at, jpeg, sequence, timeout, stop_event):
    if not 0 < len(jpeg) <= MAX_JPEG_BYTES:
        raise ValueError("JPEG 長度超出上限")
    # 期限從擷取開始計算，包含推論、佇列、傳輸與解碼，不容許舊影格續命。
    deadline = captured_at + timeout
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise TimeoutError("影格已過期")
    client.settimeout(remaining)
    client.sendall(VIDEO_HEADER.pack(b"MTV4", sequence, len(jpeg)) + jpeg)
    magic, confirmed_sequence = VIDEO_ACK.unpack(
        receive_exact(client, VIDEO_ACK.size, deadline, stop_event))
    if magic != b"MTA4" or confirmed_sequence != sequence:
        raise ValueError("影格確認編號不符")
    if time.monotonic() > deadline:
        raise TimeoutError("影格確認已過期")


def video_sender_worker(config, video_queue, stop_event, status_queue):
    network = config["network"]
    remote = (network["pc_host"], int(network["video_port"]))
    timeout = float(config.get("control", {}).get("video_timeout_s", 1.0))
    client = None
    next_retry = 0.0
    sequence = losses = 0

    def report(state, **details):
        put_latest(status_queue, dict(state=state, at=time.monotonic(), losses=losses, **details))

    try:
        while not stop_event.is_set():
            if client is None:
                if stop_event.wait(max(0.0, next_retry - time.monotonic())):
                    break
                try:
                    client = socket.create_connection(remote, timeout=1.0)
                    client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    report("connected")
                    print("[影像] 已連接 {}:{}；等待新影格確認。".format(*remote), flush=True)
                except OSError:
                    losses += 1
                    report("disconnected")
                    if client is not None:
                        client.close()
                    client = None
                    next_retry = time.monotonic() + 1.0
                    continue
            try:
                # 沒有新影格時仍觀察 TCP 關閉；單純連線存在不算影像健康。
                if select.select([client], [], [], 0)[0]:
                    raise ConnectionError("影像連線已關閉或收到非預期資料")
                try:
                    packet = video_queue.get(timeout=0.05)
                except queue.Empty:
                    continue
                captured_at, jpeg = newest_available(video_queue, packet)
                if not 0 <= time.monotonic() - captured_at < timeout:
                    continue
                sequence += 1
                transmit_frame(client, captured_at, jpeg, sequence, timeout, stop_event)
                report("confirmed", captured_at=captured_at, frame_seq=sequence)
            except (OSError, EOFError, ValueError) as exc:
                losses += 1
                report("disconnected")
                print("[影像] {}；等待重新連線。".format(exc), flush=True)
                client.close()
                client = None
                next_retry = time.monotonic() + 1.0
                # 半包不續傳，重連後只傳仍在有效期內的最新影格。
    finally:
        report("stopped")
        if client is not None:
            client.close()


def display_worker(display_queue, stop_event):
    try:
        import cv2
        while not stop_event.is_set():
            try:
                frame = display_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            cv2.imshow("YOLOv8 Seg+Track", newest_available(display_queue, frame))
            if cv2.waitKey(1) & 0xFF == ord("q"):
                stop_event.set()
                break
    except Exception as exc:
        print("[顯示] {}；無桌面環境請加 --no-display。".format(exc), flush=True)
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


def atomic_snapshot(path, snapshot):
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(snapshot, stream, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
    os.replace(str(temporary), str(path))


def combined_snapshot(store, authority, now=None, runtime=None):
    stamp = time.monotonic() if now is None else now
    result = store.snapshot(now=stamp)
    result["control"] = authority.snapshot(now=stamp)
    result["run_id"] = authority.session
    result["boot_id"] = BOOT_ID
    result["runtime"] = dict(runtime or {"state": "running", "reason": "running"})
    result["recording_thresholds_ms"] = {
        "tof": store.config.get("tof", {}).get("stale_ms", 120),
        "m4": store.config.get("tof", {}).get("telemetry_stale_ms", 250),
        "lidar": store.config.get("lidar", {}).get("stale_ms", 500),
        "camera": store.config.get("camera", {}).get("stale_ms", 1000),
    }
    return result


def parking_input(store, authority):
    """先取帶原始時間基準的感測快照，再記輸入時間；不在控制鎖內複製點雲。"""
    snapshot = store.snapshot()
    control = authority.snapshot()
    # 停車程序不讀取影像輪廓；縮小程序間傳送量，不傳 UART 物件或感測鎖。
    compact = {key: snapshot[key] for key in ("monotonic_s", "m4", "tof", "lidar")}
    return {"session": authority.session, "control": control, "snapshot": compact,
            "at": time.monotonic()}


def load_config(default_pc_host=None, argv=None):
    parser = argparse.ArgumentParser(description="MiniTesla 感測整合與控制權管理")
    parser.add_argument("--config", default=str(ROOT / "sensor_config.json"))
    parser.add_argument("--pc-host", default=os.environ.get("MINITESLA_PC_HOST"))
    parser.add_argument("--uart", help="覆寫 UART 裝置路徑")
    parser.add_argument("--model", help="覆寫模型路徑")
    parser.add_argument("--camera-index", type=int)
    parser.add_argument("--no-display", action="store_true", help="不開啟 Jetson 本地視窗")
    args = parser.parse_args(argv)
    config_path = Path(args.config).expanduser().resolve()
    with config_path.open(encoding="utf-8") as stream:
        config = json.load(stream)
    network = config["network"]
    network["pc_host"] = args.pc_host or network.get("pc_host") or default_pc_host or "120.126.10.41"
    if args.uart:
        config["uart"]["device"] = args.uart
    if args.camera_index is not None:
        config["camera"]["index"] = args.camera_index
    if args.no_display:
        config["camera"]["display"] = False
    model = Path(args.model or config["camera"]["model"]).expanduser()
    config["camera"]["model"] = str(model if model.is_absolute() else config_path.parent / model)
    output = Path(config["snapshot"]["path"]).expanduser()
    config["snapshot"]["path"] = str(output if output.is_absolute() else config_path.parent / output)
    if int(config["uart"]["baud"]) != 115200:
        parser.error("本次 M4 韌體使用 115200；請勿只修改單端鮑率。")
    from minitesla_sensors.parking_config import load_settings
    try:
        settings = load_settings()
    except ValueError as exc:
        parser.error(str(exc))
    config["auto_parking"] = settings
    if settings["configuration_complete"]:
        lidar = config.setdefault("lidar", {})
        lidar["yaw_deg"] = settings["lidar_yaw_deg"]
        lidar["clockwise"] = settings["lidar_clockwise"]
        position = lidar.setdefault("position_mm", {})
        position["x"] = settings["lidar_x_from_tail_m"] * 1000
        position["y"] = settings["lidar_y_from_center_m"] * 1000
    return config


def main(default_pc_host=None, argv=None):
    config = load_config(default_pc_host, argv)
    # 鎖涵蓋整個生命週期；不同資料夾／設定檔啟動也不能搶同一台車。
    # spawn 子行程只匯入本檔，不會執行 main 或繼承這個非繼承描述元。
    instance = SingleInstance()
    try:
        instance.acquire()
    except SingleInstanceError as exc:
        raise SystemExit(str(exc))
    previous_handlers = {}
    shutdown = ShutdownRequest()
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, shutdown.request)
        return run_vehicle(config, shutdown=shutdown)
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        instance.release()


def run_vehicle(config, shutdown=None):
    shutdown = shutdown or ShutdownRequest()
    if not Path(config["camera"]["model"]).is_file():
        raise SystemExit("找不到模型：{}；請放入既有 best.pt 或指定 --model。".format(config["camera"]["model"]))
    output = Path(config["snapshot"]["path"])
    output.parent.mkdir(parents=True, exist_ok=True)
    store = SensorStore(config)
    authority = ControlAuthority(config)
    runtime = {"state": "starting", "reason": "starting"}
    atomic_snapshot(output, combined_snapshot(store, authority, runtime=runtime))
    ctx = multiprocessing.get_context("spawn")
    stop_event = ctx.Event()
    metadata_queue = ctx.Queue(maxsize=2)
    video_queue = ctx.Queue(maxsize=3)
    video_status_queue = ctx.Queue(maxsize=8)
    display_queue = ctx.Queue(maxsize=1) if config["camera"].get("display", True) else None
    auto_enabled = config.get("auto_parking", {}).get("enabled") is True
    parking_input_queue = ctx.Queue(maxsize=2) if auto_enabled else None
    parking_output_queue = ctx.Queue(maxsize=2) if auto_enabled else None
    parking_process = None
    processes = [ctx.Process(name="camera_inference", target=camera_worker,
                             args=(config, metadata_queue, video_queue, display_queue, stop_event)),
                 ctx.Process(name="video_sender", target=video_sender_worker,
                             args=(config, video_queue, stop_event, video_status_queue))]
    if display_queue is not None:
        processes.append(ctx.Process(name="local_display", target=display_worker,
                                     args=(display_queue, stop_event)))
    if auto_enabled:
        from minitesla_sensors.parking_auto import parking_worker
        parking_process = ctx.Process(name="parking_planner", target=parking_worker,
                                      args=(config, parking_input_queue, parking_output_queue, stop_event))
        processes.append(parking_process)
    uart = UartBridge(store, config, stop_event, authority)
    lidar = LidarReceiver(store, config, stop_event)
    exit_code, stop_reason = 0, "local_stop"
    period = max(0.05, float(config["snapshot"].get("period_s", 0.1)))
    try:
        # 使用 spawn 並先建立子程序，不讓相機或影像程序繼承 UART 檔案描述元。
        for process in processes:
            if shutdown.requested:
                break
            process.start()
        if not shutdown.requested:
            uart.start()
            lidar.start()
        runtime = {"state": "running", "reason": "running"}
        print("[感測整合] 狀態快照：{}；Ctrl+C 結束。".format(output), flush=True)
        next_snapshot = time.monotonic()
        next_parking = next_snapshot
        while not stop_event.is_set() and not shutdown.requested:
            for _ in range(16):
                try:
                    report = video_status_queue.get_nowait()
                except queue.Empty:
                    break
                authority.update_video(report)
            if processes[1].exitcode is not None:
                authority.update_video({"state": "stopped", "at": time.monotonic(),
                                        "losses": authority.snapshot()["video"]["disconnect_count"]})
                exit_code, stop_reason = 1, "video_worker_exit"
                break
            for _ in range(8):
                try:
                    message = metadata_queue.get_nowait()
                except queue.Empty:
                    break
                if message["kind"] == "camera":
                    store.update_camera(message["detections"], message["frame_size"],
                                        captured_at=message["captured_at"], received_at=time.monotonic())
                else:
                    store.set_source_error("camera", message["message"])
            if processes[0].exitcode is not None:
                store.set_source_error("camera", "影像推論程序已結束；請檢查相機與模型後重新啟動")
                exit_code, stop_reason = 1, "camera_worker_exit"
                break
            if not uart.is_alive():
                exit_code, stop_reason = 1, "m4_transport_error"
                break
            if not lidar.is_alive():
                exit_code, stop_reason = 1, "lidar_receiver_exit"
                break
            if parking_process is not None:
                if parking_process.exitcode is not None:
                    exit_code, stop_reason = 1, "parking_worker_exit"
                    break
                for _ in range(4):
                    try:
                        proposal = parking_output_queue.get_nowait()
                    except queue.Empty:
                        break
                    if isinstance(proposal, dict) and proposal.get("kind") == "mission":
                        authority.update_mission(proposal)
                    else:
                        authority.update_parking(proposal)
            now = time.monotonic()
            if parking_process is not None and now >= next_parking:
                put_latest(parking_input_queue, parking_input(store, authority))
                next_parking = now + 0.1
            if now >= next_snapshot:
                atomic_snapshot(output, combined_snapshot(store, authority, now=now, runtime=runtime))
                next_snapshot = now + period
            stop_event.wait(0.01)
    except KeyboardInterrupt:
        shutdown.request(signal.SIGINT)
        print("\n[主程式] 正在停止。", flush=True)
    except Exception:
        exit_code, stop_reason = 1, "main_exception"
        traceback.print_exc()
    finally:
        # UART 可能在主迴圈第一次執行之前便失敗並設 stop_event。
        if uart.failure is not None:
            exit_code, stop_reason = 1, "m4_transport_error"
        elif not exit_code and shutdown.requested:
            stop_reason = shutdown.reason
        authority.fault(stop_reason)
        stop_event.set()
        runtime = {"state": "failed" if exit_code else "stopping", "reason": stop_reason}
        try:
            atomic_snapshot(output, combined_snapshot(store, authority, runtime=runtime))
        except Exception as exc:
            print("[感測整合] 停止快照無法寫入：{}".format(exc), flush=True)
        for worker in (uart, lidar):
            if worker.ident is not None:
                worker.join(timeout=1.0)
        for process in processes:
            if process.pid is None:
                continue
            process.join(timeout=1.0)
            if process.is_alive():
                process.terminate()
                process.join(timeout=1.0)
            if process.is_alive():
                process.kill()
                process.join(timeout=1.0)
        if any(worker.is_alive() for worker in (uart, lidar)) or any(
                process.pid is not None and process.is_alive() for process in processes):
            exit_code = 1
            if stop_reason in ("local_stop", "signal_sigint", "signal_sigterm"):
                stop_reason = "cleanup_timeout"
                authority.fault(stop_reason)
            print("[主程式] 工作項目未在期限內結束。", flush=True)
        # 保留原始來源錯誤；終止狀態另外明確標記，紀錄器不再視為即時資料。
        existing_errors = store.snapshot().get("errors", {})
        for source in ("m4", "lidar", "camera"):
            if source not in existing_errors:
                store.set_source_error(source, "主程式已停止")
        runtime = {"state": "failed" if exit_code else "stopped", "reason": stop_reason}
        try:
            atomic_snapshot(output, combined_snapshot(store, authority, runtime=runtime))
        except Exception as exc:
            exit_code = 1
            print("[感測整合] 結束快照無法寫入：{}".format(exc), flush=True)
        for transfer_queue in (metadata_queue, video_queue, display_queue, video_status_queue,
                               parking_input_queue, parking_output_queue):
            if transfer_queue is not None:
                transfer_queue.cancel_join_thread()
                transfer_queue.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
