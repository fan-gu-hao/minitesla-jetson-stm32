#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""統一操作視窗的影像與雷達背景程序；不在此處操作 Tk 元件。

影像沿用 pc_receive.py 的 MTV4 / MTA4 協定。只有主視窗確認已呈現
對應影格後，才允許該協定回覆 Jetson。背景程序失效會通知主程序停用控制。
"""
import math
import queue
import socket
import threading
import time
import uuid

from pc_receive import receive_video_frame

DISPLAY_DEADLINE_S = 0.75
MAX_SOURCE_PIXELS = 16 * 1024 * 1024
MAX_SOURCE_DIMENSION = 8192
LIDAR_RANGE_MM = 6000
LIDAR_TTL_S = 0.5
LIDAR_EMIT_S = 0.1


def _listener_options(sock, tcp=False):
    # Windows 的 REUSEADDR 允許第二個接收器搶占同埠，必須改用排他綁定。
    exclusive = getattr(socket, 'SO_EXCLUSIVEADDRUSE', None)
    if exclusive is not None:
        sock.setsockopt(socket.SOL_SOCKET, exclusive, 1)
    elif tcp:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)


def _put_latest(events, event):
    """有界佇列滿時移除舊項目；不因介面停頓永久卡住工作執行緒。"""
    for _ in range(3):
        try:
            events.put_nowait(event)
            return True
        except queue.Full:
            try:
                # multiprocessing.Queue 的 feeder 可能尚未讓資料可讀。
                events.get(timeout=0.01)
            except queue.Empty:
                pass
    return False


def _jpeg_dimensions(jpeg):
    """在解碼配置記憶體前限制 JPEG 宣告尺寸，拒絕截斷或異常標頭。"""
    if not isinstance(jpeg, bytes) or not jpeg.startswith(b'\xff\xd8'):
        raise ValueError('影像不是有效的 JPEG')
    offset = 2
    sof = {0xc0, 0xc1, 0xc2, 0xc3, 0xc5, 0xc6, 0xc7,
           0xc9, 0xca, 0xcb, 0xcd, 0xce, 0xcf}
    while offset < len(jpeg):
        if jpeg[offset] != 0xff:
            raise ValueError('JPEG 標頭格式錯誤')
        while offset < len(jpeg) and jpeg[offset] == 0xff:
            offset += 1
        if offset >= len(jpeg):
            break
        marker = jpeg[offset]
        offset += 1
        if marker in (0xda, 0xd9):
            break
        if marker == 0x01 or 0xd0 <= marker <= 0xd8:
            continue
        if marker == 0 or offset + 2 > len(jpeg):
            break
        size = int.from_bytes(jpeg[offset:offset + 2], 'big')
        if size < 2 or offset + size > len(jpeg):
            break
        if marker in sof:
            if size < 8:
                break
            height = int.from_bytes(jpeg[offset + 3:offset + 5], 'big')
            width = int.from_bytes(jpeg[offset + 5:offset + 7], 'big')
            if (width <= 0 or height <= 0 or
                    max(width, height) > MAX_SOURCE_DIMENSION or
                    width * height > MAX_SOURCE_PIXELS):
                raise ValueError('JPEG 尺寸超出接收上限')
            return width, height
        offset += size
    raise ValueError('JPEG 缺少完整尺寸標頭')


def decode_jpeg_ppm(jpeg):
    """只由背景程序呼叫；沿用 OpenCV / NumPy，無須額外影像套件。"""
    width, height = _jpeg_dimensions(jpeg)
    import cv2
    import numpy as np

    frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None or frame.ndim != 3 or frame.shape[:2] != (height, width):
        raise ValueError('JPEG 無法完整解碼或尺寸不一致')
    scale = min(1.0, 640.0 / width, 360.0 / height)
    draw_width = max(1, int(width * scale))
    draw_height = max(1, int(height * scale))
    if (draw_width, draw_height) != (width, height):
        frame = cv2.resize(frame, (draw_width, draw_height), interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    ppm = ('P6\n{} {}\n255\n'.format(draw_width, draw_height).encode('ascii') +
           rgb.tobytes())
    return ppm, draw_width, draw_height


def _video_loop(bind, port, events, display_acks, stop_event, decoder=None):
    decoder = decoder or decode_jpeg_ppm
    next_frame_id = 0

    def state(value, connection_id='', message=''):
        _put_latest(events, {'kind': 'video_state', 'state': value,
                            'connection_id': connection_id, 'message': message,
                            'at': time.monotonic()})

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        _listener_options(server, tcp=True)
        server.bind((bind, port))
        server.listen(2)
        server.settimeout(0.1)
        state('listening', message='等待車端影像連線')
        while not stop_event.is_set():
            try:
                client, address = server.accept()
            except socket.timeout:
                continue
            connection_id = uuid.uuid4().hex
            state('connected', connection_id, '車端影像已連線')
            with client:
                client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

                def show_frame(jpeg):
                    nonlocal next_frame_id
                    received_at = time.monotonic()
                    deadline = received_at + DISPLAY_DEADLINE_S
                    ppm, width, height = decoder(jpeg)
                    if stop_event.is_set():
                        return False
                    if time.monotonic() >= deadline:
                        raise TimeoutError('影像解碼逾時')
                    next_frame_id += 1
                    frame_id = next_frame_id
                    if not _put_latest(events, {
                            'kind': 'video_frame', 'connection_id': connection_id,
                            'frame_id': frame_id, 'received_at': received_at,
                            'deadline': deadline, 'ppm': ppm,
                            'width': width, 'height': height}):
                        raise OSError('影像顯示佇列無法接收')
                    while not stop_event.is_set():
                        remaining = deadline - time.monotonic()
                        if remaining <= 0:
                            raise TimeoutError('操作視窗未在期限內呈現影像')
                        try:
                            ack = display_acks.get(timeout=min(0.05, remaining))
                        except queue.Empty:
                            continue
                        if (isinstance(ack, dict) and
                                ack.get('connection_id') == connection_id and
                                ack.get('frame_id') == frame_id and
                                not stop_event.is_set() and
                                time.monotonic() < deadline):
                            return True
                    return False

                sequence = 0
                try:
                    while not stop_event.is_set():
                        sequence = receive_video_frame(
                            client, sequence, show_frame, stop_event)
                except InterruptedError:
                    if not stop_event.is_set():
                        state('disconnected', connection_id, '影像顯示已中斷，等待重新連線')
                except Exception as exc:
                    # 解碼錯誤／半包／序號錯誤只關閉本次連線；不能誤回覆確認。
                    if not stop_event.is_set():
                        state('disconnected', connection_id,
                              '影像中斷：{}；等待重新連線'.format(exc))
                finally:
                    if stop_event.is_set():
                        state('disconnected', connection_id, '影像接收已停止')


class LidarBins:
    """原始角度的一度一點顯示快取；不推定車頭方向。"""
    def __init__(self):
        self.points = {}
        self.last_valid_at = None

    def accept(self, data, now):
        try:
            parts = data.decode('ascii').strip().split(',')
            if len(parts) != 3:
                return False
            angle, distance = float(parts[0]), float(parts[1])
            quality = int(parts[2])
        except (ValueError, UnicodeError, AttributeError):
            return False
        if (not math.isfinite(angle) or not math.isfinite(distance) or
                not 0 <= angle < 360 or not 0 < distance <= LIDAR_RANGE_MM or
                not 1 <= quality <= 255):
            return False
        self.points[int(angle)] = [angle, distance, now]
        self.last_valid_at = now
        return True

    def snapshot(self, now):
        self.points = {key: point for key, point in self.points.items()
                       if 0 <= now - point[2] < LIDAR_TTL_S}
        valid = bool(self.points)
        return {'kind': 'lidar_scan',
                'points': [self.points[key] for key in sorted(self.points)],
                'at': now, 'last_packet_at': self.last_valid_at,
                'last_valid_at': self.last_valid_at, 'valid': valid,
                'message': '雷達資料接收中' if valid else '等待有效雷達資料',
                'range_mm': LIDAR_RANGE_MM}


def _lidar_loop(bind, port, events, stop_event):
    bins = LidarBins()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        _listener_options(sock)
        sock.bind((bind, port))
        sock.settimeout(0.02)
        next_emit = 0.0
        while not stop_event.is_set():
            # 一批最多 32 包，且到發佈期限就讓出，避免持續封包淹沒介面更新。
            for _ in range(32):
                if stop_event.is_set():
                    break
                try:
                    packet, address = sock.recvfrom(1024)
                except socket.timeout:
                    break
                bins.accept(packet, time.monotonic())
                if time.monotonic() >= next_emit:
                    break
            now = time.monotonic()
            if now >= next_emit:
                _put_latest(events, bins.snapshot(now))
                next_emit = now + LIDAR_EMIT_S


def media_worker(bind, video_port, lidar_port, video_events, lidar_events,
                 display_acks, stop_event):
    """spawn 子程序入口：兩條 I/O 執行緒，任何致命失敗皆停止整個介面控制。"""
    failures = []
    failure_lock = threading.Lock()

    def guarded(name, target, args):
        try:
            target(*args)
            if not stop_event.is_set():
                raise RuntimeError('{}接收執行緒非預期結束'.format(name))
        except Exception as exc:
            with failure_lock:
                failures.append('{}：{}'.format(name, exc))
            _put_latest(video_events, {
                'kind': 'video_state', 'state': 'error', 'connection_id': '',
                'message': '{}接收失敗：{}'.format(name, exc), 'at': time.monotonic()})
            stop_event.set()

    threads = [
        threading.Thread(target=guarded, args=(
            '影像', _video_loop,
            (bind, video_port, video_events, display_acks, stop_event)), daemon=True),
        threading.Thread(target=guarded, args=(
            '雷達', _lidar_loop,
            (bind, lidar_port, lidar_events, stop_event)), daemon=True),
    ]
    try:
        for thread in threads:
            thread.start()
        while not stop_event.wait(0.1):
            if not all(thread.is_alive() for thread in threads):
                with failure_lock:
                    failures.append('媒體接收執行緒已停止')
                stop_event.set()
    except KeyboardInterrupt:
        stop_event.set()
    finally:
        stop_event.set()
        for thread in threads:
            if thread.ident is not None:
                thread.join(timeout=1.0)
        for events in (video_events, lidar_events):
            cancel = getattr(events, 'cancel_join_thread', None)
            if cancel is not None:
                cancel()
    if failures:
        raise SystemExit(1)
