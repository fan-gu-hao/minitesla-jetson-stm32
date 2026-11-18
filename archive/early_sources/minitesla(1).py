# -*- coding: utf-8 -*-
from ultralytics import YOLO
import cv2
import numpy as np
import spidev
import time
import Jetson.GPIO as GPIO  # 用於 Jetson Orin Nano 控制 GPIO
import socket
from multiprocessing import Process, Queue as mpqueue
import threading
import pickle
import struct
import queue
import spidev
import Jetson.GPIO as GPIO
import serial

import socket
import serial
from itertools import count
from datetime import datetime



def start_jetson_udp_client_uart(pc_host: str = '192.0.2.10',
                                 pc_port: int = 5000,
                                 serdev: str = '/dev/ttyTHS0',
                                 baud: int = 115200,
                                 log_every: int = 1,
                                 hello_interval: float = 2.0):
    """
    啟動 Jetson 端 UDP Client：
      1) 週期性對 PC 發 'HELLO'
      2) 接收 PC 的 2-byte 指令 [speed, turn]
      3) 封成 [55 AA 02 SEQ speed turn CRC8] 從 UART 丟到 STM32
    回傳: (stop_event, thread)
    """
    stop_evt = threading.Event()

    def crc8(data: bytes, poly=0x07) -> int:
        c = 0
        for b in data:
            c ^= b
            for _ in range(8):
                c = ((c << 1) ^ poly) & 0xFF if (c & 0x80) else ((c << 1) & 0xFF)
        return c

    def build_uart(speed: int, turn: int, seq: int) -> bytes:
        head = bytes([0x55, 0xAA, 0x02, seq & 0xFF, speed & 0xFF, turn & 0xFF])
        return head + bytes([crc8(head)])

    def hexstr(b: bytes) -> str:
        return ' '.join(f'{x:02X}' for x in b)

    def _worker():
        # UART
        try:
            ser = serial.Serial(
                serdev, baud,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                timeout=0, write_timeout=0, rtscts=False, dsrdtr=False, xonxoff=False
            )
            ser.reset_output_buffer(); ser.reset_input_buffer()
        except Exception as e:
            raise RuntimeError(f"無法開啟 UART {serdev}@{baud}: {e}")

        # UDP client
        us = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        us.settimeout(0.5)
        print(f"[JETSON] UDP client → {pc_host}:{pc_port}  |  UART {serdev}@{baud}")

        seq_gen = count(0)
        packets = 0
        last_hello = 0.0

        try:
            while not stop_evt.is_set():
                now = time.time()
                if now - last_hello > hello_interval:
                    try:
                        us.sendto(b'HELLO', (pc_host, pc_port))
                    except Exception as e:
                        print(f"[JETSON] HELLO send error: {e}")
                    last_hello = now

                try:
                    data, addr = us.recvfrom(64)  # 收兩個 byte（保留 buffer 餘裕）
                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"[JETSON] UDP recv error: {e}")
                    time.sleep(0.05)
                    continue

                if len(data) < 2:
                    continue

                speed, turn = data[0], data[1]
                seq = next(seq_gen)
                pkt = build_uart(speed, turn, seq)
                try:
                    n = ser.write(pkt)
                except Exception as e:
                    print(f"[JETSON] UART write error: {e}")
                    n = 0

                packets += 1
                if log_every and (packets % log_every == 0):
                    ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                    print(f"[{ts}] RX_UDP {addr[0]}:{addr[1]}  speed={speed:3d}  turn={turn:3d}  |  "
                          f"TX_UART seq={seq:5d} bytes={n} pkt=[{hexstr(pkt)}]", flush=True)
        finally:
            try: us.close()
            except: pass
            try: ser.close()
            except: pass
            print("[JETSON] thread exit")

    th = threading.Thread(target=_worker, name="jetson_udp_client_uart", daemon=True)
    th.start()
    return stop_evt, th












def lidar_forwarding_loop():
 
    LOCAL_IP = "127.0.0.1"
    LOCAL_PORT = 6000


    REMOTE_IP = "192.0.2.10"
    REMOTE_PORT = 6000 


    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.bind((LOCAL_IP, LOCAL_PORT))


    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"✅ [LiDAR] Listening on {LOCAL_IP}:{LOCAL_PORT} and forwarding to {REMOTE_IP}:{REMOTE_PORT} ...")

    while True:
        data, addr = recv_sock.recvfrom(1024)
        try:
            decoded = data.decode().strip()
            angle, distance, quality = decoded.split(',')
            angle = float(angle)
            distance = float(distance)
            quality = int(quality)

            #print(f"📡 [LiDAR] Received → Angle: {angle:.2f}°  Distance: {distance:.2f} mm  Quality: {quality}")

            
            send_sock.sendto(data, (REMOTE_IP, REMOTE_PORT))

        except Exception as e:
            print(f"⚠️ [LiDAR] Error decoding data: {data} — {e}")


 

def video_send_stream(video_sq, video_dispq, videotext_q):
    CLASS_NAMES = [  # 原始類別名照舊
        "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
        "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
        "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
        "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
        "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
        "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl",
        "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza",
        "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet",
        "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
        "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
        "hair drier", "toothbrush", "vacant"
    ]
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 99]
    model = YOLO("best.pt")
    cap = cv2.VideoCapture(0)

    def replace_latest(queue, item):
        try:
            queue.get_nowait()  # 丟掉舊的
        except:
            pass
        try:
            queue.put_nowait(item)  # 放入新的
        except:
            pass




    def get_color(idx):
        """根據 ID 生成獨特 RGB 顏色"""
        np.random.seed(idx)
        return tuple(int(x) for x in np.random.randint(0, 255, 3))

    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            break

        # YOLOv8 tracking + segmentation 推理
        results = model.track(
            source=frame,
            tracker="bytetrack.yaml",
            persist=True,
            stream=True,
            conf=0.25,  # YOLOv8 預設
            iou=0.7     # YOLOv8 預設
        )

        for r in results:
            boxes = r.boxes
            ids = r.boxes.id if r.boxes.id is not None else []

            # ➤ segmentation 遮罩繪製（每個用不同顏色）
            if hasattr(r, "masks") and r.masks is not None:
                for i, seg in enumerate(r.masks.xy):
                    color = get_color(int(ids[i]) if len(ids) > i else i)
                    pts = seg.reshape(-1, 1, 2).astype(int)
                    cv2.polylines(frame, [pts], isClosed=True,
                                  color=color, thickness=2)

            for i, box in enumerate(boxes):
                xyxy = box.xyxy[0].cpu().numpy()
                conf = box.conf[0].item()
                cls = int(box.cls[0])
                class_name = CLASS_NAMES[cls] if cls < len(
                    CLASS_NAMES) else f"class_{cls}"
                track_id = int(ids[i]) if len(ids) > i else -1
                color = get_color(track_id if track_id != -1 else cls)

                # ➤ 終端輸出
                #print(f"[ID {track_id}] 類別={class_name} | 信心={conf:.2f} | 框={xyxy.astype(int)}")

                # ➤ 畫框與標籤
                cv2.rectangle(frame,
                              (int(xyxy[0]), int(xyxy[1])),
                              (int(xyxy[2]), int(xyxy[3])),
                              color, 2)
                cv2.putText(frame,
                            f"ID {track_id} {class_name} {conf:.2f}",
                            (int(xyxy[0]), int(xyxy[1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            color, 1)
        
        replace_latest(video_dispq, frame)
        result, image = cv2.imencode('.jpg', frame, encode_param)
        #data = pickle.dumps(image, 0)
        data = pickle.dumps(image, protocol=pickle.HIGHEST_PROTOCOL)
        size = len(data)
        video_data = struct.pack(">L", size) + data
        
        replace_latest(video_sq, video_data)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()


def video_display(video_dispq):
    while True:
        if not video_dispq.empty():
            frame = video_dispq.get()  # 從佇列中取得影像
            cv2.imshow('YOLOv8 Seg+Track', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    cv2.destroyAllWindows()


""" 這邊是資料輸入  """


def video_send_tcp(video_sq):
    ip='192.0.2.10'
    port=7000
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((ip, port))
    print(f"✅ Connected to video TCP server at {ip}:{port}")

    try:
        while True:
            if not video_sq.empty():
                data = video_sq.get()
                client_socket.sendall(data)
    except Exception as e:
        print(f"❌ Video TCP send error: {e}")
    finally:
        client_socket.close()





if __name__ == "__main__":
    # 先啟動「UDP(PC)->UART(STM32)」背景執行緒
    stop_evt, motor_thread = start_jetson_udp_client_uart(
        pc_host='192.0.2.10',  # ← PC 的 IP
        pc_port=5000,
        serdev='/dev/ttyTHS0',
        baud=115200,
        log_every=1
    )

    # 啟動你原本的多進程任務
    video_dispq = mpqueue(maxsize=1)
    video_sq = mpqueue(maxsize=3)
    videotext_q = mpqueue(maxsize=1)
    ctrl_q = mpqueue()

    display_process = Process(target=video_display, args=(video_dispq,))
    video_send_stream_process = Process(target=video_send_stream, args=(video_sq, video_dispq, videotext_q,))
    video_send_process = Process(target=video_send_tcp, args=(video_sq,))
    lidar_stream = Process(target=lidar_forwarding_loop)

    procs = [lidar_stream, video_send_stream_process, video_send_process, display_process]

    try:
        for p in procs:
            p.start()

        # 讓主執行緒常駐；Ctrl+C 會走到 except
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n[MAIN] KeyboardInterrupt, shutting down...")

    finally:
        # 停掉 UART 背景執行緒
        try:
            stop_evt.set()
            motor_thread.join(timeout=2)
        except Exception:
            pass

        # 結束所有子進程
        for p in procs:
            try:
                if p.is_alive():
                    p.terminate()
            except Exception:
                pass
        for p in procs:
            try:
                p.join(timeout=2)
            except Exception:
                pass

        print("[MAIN] all done.")




