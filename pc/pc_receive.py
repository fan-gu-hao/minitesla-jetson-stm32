#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第四階段影像／雷達接收器，取代 from_jetson_orin_nano(1).py。

影像採原始 JPEG 與逐影格確認；斷線後重新等待 Jetson 連線。
依賴沿用原本的 OpenCV、NumPy；影像與雷達分開程序顯示。
"""
import argparse
import math
import multiprocessing
import socket
import struct
import time

VIDEO_HEADER = struct.Struct("!4sQI")
VIDEO_ACK = struct.Struct("!4sQ")
MAX_JPEG_BYTES = 8 * 1024 * 1024


def receive_exact(client, size, deadline, stop_event):
    data = bytearray()
    while len(data) < size:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or stop_event.is_set():
            raise TimeoutError("影像資料逾時或停止")
        client.settimeout(min(0.2, remaining))
        try:
            chunk = client.recv(min(65536, size - len(data)))
        except socket.timeout:
            continue
        if not chunk:
            raise EOFError("Jetson 已關閉影像連線")
        data.extend(chunk)
    return bytes(data)


def receive_video_frame(client, last_sequence, show_frame, stop_event):
    """只在完整影格解碼及顯示函式成功後回覆；半包不可當成成功。"""
    deadline = time.monotonic() + 2.0
    magic, sequence, size = VIDEO_HEADER.unpack(
        receive_exact(client, VIDEO_HEADER.size, deadline, stop_event))
    if magic != b"MTV4" or sequence <= last_sequence or not 0 < size <= MAX_JPEG_BYTES:
        raise ValueError("影像版本、序號或長度錯誤；請確認兩端皆為第四階段")
    jpeg = receive_exact(client, size, deadline, stop_event)
    if not show_frame(jpeg):
        raise InterruptedError("影像視窗已關閉")
    client.settimeout(0.5)
    client.sendall(VIDEO_ACK.pack(b"MTA4", sequence))
    return sequence


def video_receive_stream(bind, port, stop_event):
    import cv2
    import numpy as np

    title = "Smart Car | Video"

    def show_frame(jpeg):
        frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError("JPEG 無法解碼")
        cv2.imshow(title, frame)
        if cv2.waitKey(1) & 0xff in (ord('q'), 27):
            stop_event.set()
            return False
        if cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
            stop_event.set()
            return False
        return True

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((bind, port))
            server.listen(2)
            server.settimeout(0.2)
            print("[影像] 等待 Jetson 連入 {}:{}。".format(bind, port), flush=True)
            while not stop_event.is_set():
                try:
                    client, address = server.accept()
                except socket.timeout:
                    continue
                with client:
                    client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    print("[影像] 已連線：{}".format(address), flush=True)
                    sequence = 0
                    try:
                        while not stop_event.is_set():
                            sequence = receive_video_frame(client, sequence, show_frame, stop_event)
                    except InterruptedError:
                        stop_event.set()
                    except (OSError, EOFError, ValueError, cv2.error) as exc:
                        if not stop_event.is_set():
                            print("[影像] {}；重新等待連線。".format(exc), flush=True)
                    finally:
                        # 中斷即移除舊畫面，避免把停住的影像當成即時畫面。
                        cv2.destroyAllWindows()
    finally:
        cv2.destroyAllWindows()


def lidar_receive(bind, port, stop_event):
    import cv2
    import numpy as np

    width = height = 800
    center = (width // 2, height // 2)
    radius_mm = 6000
    scale = width / (2 * radius_mm)
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    points = []
    next_draw = 0.0
    last_data = None
    title = "Smart Car | LiDAR"
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind((bind, port))
            sock.settimeout(0.02)
            print("[雷達] 等待資料 {}:{}。".format(bind, port), flush=True)
            while not stop_event.is_set():
                now = time.monotonic()
                try:
                    data, _ = sock.recvfrom(1024)
                except socket.timeout:
                    data = None
                if data is not None:
                    try:
                        a, d, q = data.decode().strip().split(',')
                        angle, distance, quality = float(a), float(d), int(q)
                        if not math.isfinite(angle) or not math.isfinite(distance):
                            continue
                        last_data = time.monotonic()
                        if 0 < distance < radius_mm and quality > 0:
                            theta, r = math.radians(angle), distance * scale
                            points.append((int(center[0] + r * math.cos(theta)),
                                           int(center[1] - r * math.sin(theta))))
                            if len(points) > 4096:
                                del points[:2048]
                    except (ValueError, UnicodeError):
                        pass
                if now >= next_draw:
                    frame[:] = 0
                    cv2.line(frame, (center[0], 0), (center[0], height), (50, 50, 50), 1)
                    cv2.line(frame, (0, center[1]), (width, center[1]), (50, 50, 50), 1)
                    for point in points:
                        cv2.circle(frame, point, 2, (255, 100, 0), -1)
                    if last_data is None or now-last_data > 0.5:
                        cv2.putText(frame, "Waiting for LiDAR", (20, 35), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.7, (0, 180, 255), 2)
                    cv2.imshow(title, frame)
                    points.clear()
                    next_draw = now + 0.1
                    if cv2.waitKey(1) & 0xff in (ord('q'), 27):
                        stop_event.set()
                    elif cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                        stop_event.set()
    finally:
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="智慧車第四階段影像及雷達接收器")
    parser.add_argument('--bind', default='0.0.0.0')
    parser.add_argument('--video-port', type=int, default=7000)
    parser.add_argument('--lidar-port', type=int, default=6000)
    args = parser.parse_args()
    if not all(1 <= port <= 65535 for port in (args.video_port, args.lidar_port)):
        parser.error('連接埠必須介於 1～65535')
    ctx = multiprocessing.get_context('spawn')
    stop_event = ctx.Event()
    processes = [ctx.Process(target=video_receive_stream, args=(args.bind, args.video_port, stop_event)),
                 ctx.Process(target=lidar_receive, args=(args.bind, args.lidar_port, stop_event))]
    try:
        for process in processes:
            process.start()
        while not stop_event.wait(0.1):
            if any(process.exitcode is not None for process in processes):
                print('接收程序已結束；請檢查上方訊息後重新啟動。', flush=True)
                break
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        for process in processes:
            if process.pid is None:
                continue
            process.join(timeout=1.0)
            if process.is_alive():
                process.terminate()
                process.join(timeout=1.0)


if __name__ == '__main__':
    multiprocessing.freeze_support()
    main()
