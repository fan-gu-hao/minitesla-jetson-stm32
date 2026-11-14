# -*- coding: utf-8 -*-
import socket
import struct
import pickle
import threading
import time
import numpy as np
import math
import cv2
import traceback
import sys
from multiprocessing import Process


# ────────────────────────────────────────────────
# 影像接收（維持你原本）
def video_receive_stream():
    HOST = '0.0.0.0'
    VIDEO_RECEIVE_PORT = 7000
    server_socket_video_receive = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket_video_receive.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket_video_receive.bind((HOST, VIDEO_RECEIVE_PORT))
    server_socket_video_receive.listen(5)
    print("📹 Waiting for video connection...")
    video_receive_conn, video_receive_addr = server_socket_video_receive.accept()
    print(f"🎥 Video connected from {video_receive_addr}")
    data = b""
    payload_size = struct.calcsize(">L")
    while True:
        while len(data) < payload_size:
            packet = video_receive_conn.recv(4096)
            if not packet: break
            data += packet
        if not data: break
        packed_msg_size = data[:payload_size]; data = data[payload_size:]
        msg_size = struct.unpack(">L", packed_msg_size)[0]
        while len(data) < msg_size:
            data += video_receive_conn.recv(4096)
        frame_data = data[:msg_size]; data = data[msg_size:]
        frame = pickle.loads(frame_data, fix_imports=True, encoding="bytes")
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
        cv2.imshow('server_receive', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
    video_receive_conn.close(); cv2.destroyAllWindows()

# ────────────────────────────────────────────────
# LiDAR 接收（維持你原本）
def lidar_receive():
    UDP_IP = "0.0.0.0"; UDP_PORT = 6000
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"📡 LiDAR listening on {UDP_IP}:{UDP_PORT} ...")
    WIDTH, HEIGHT = 800, 800; CENTER = (WIDTH//2, HEIGHT//2)
    MAX_RADIUS_MM = 6000; PIXELS_PER_MM = WIDTH / (2 * MAX_RADIUS_MM)
    frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8); points=[]
    while True:
        data, addr = sock.recvfrom(1024)
        try:
            decoded = data.decode().strip()
            angle, distance, quality = decoded.split(',')
            angle = float(angle); distance = float(distance); quality = int(quality)
            if 0 < distance < MAX_RADIUS_MM and quality > 0:
                theta = math.radians(angle); r = distance * PIXELS_PER_MM
                x = int(CENTER[0] + r * math.cos(theta))
                y = int(CENTER[1] - r * math.sin(theta))
                points.append((x, y))
            if angle < 1.0 and len(points) > 100:
                frame[:] = 0
                cv2.line(frame, (CENTER[0],0), (CENTER[0],HEIGHT), (50,50,50), 1)
                cv2.line(frame, (0,CENTER[1]), (WIDTH,CENTER[1]), (50,50,50), 1)
                for pt in points: cv2.circle(frame, pt, 2, (255,100,0), -1)
                cv2.imshow("LiDAR Radar (OpenCV)", frame)
                if cv2.waitKey(1) == 27: break
                points.clear()
        except Exception as e:
            print(f"⚠️ Error decoding data: {data} — {e}")
    sock.close(); cv2.destroyAllWindows()

# ────────────────────────────────────────────────
# 主程式：子行程跑影像/雷達；主線程跑鍵盤→UDP
if __name__ == "__main__":
    # 啟動子行程
    lidar_receive_process = Process(target=lidar_receive)
    video_receive_process = Process(target=video_receive_stream)
    lidar_receive_process.start()
    video_receive_process.start()

