import socket

# 本地端接收 LiDAR 資料的設定
LOCAL_IP = "127.0.0.1"
LOCAL_PORT = 6000

# 遠端伺服器設定（目標）
REMOTE_IP = "192.0.2.10"
REMOTE_PORT = 6000  # 可根據伺服器設定修改

# 建立 UDP socket（接收用）
recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.bind((LOCAL_IP, LOCAL_PORT))

# 建立 UDP socket（傳送用）
send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"✅ Listening on {LOCAL_IP}:{LOCAL_PORT} and forwarding to {REMOTE_IP}:{REMOTE_PORT} ...")

while True:
    data, addr = recv_sock.recvfrom(1024)
    try:
        decoded = data.decode().strip()
        angle, distance, quality = decoded.split(',')
        angle = float(angle)
        distance = float(distance)
        quality = int(quality)

        print(f"📡 Received → Angle: {angle:.2f}°  Distance: {distance:.2f} mm  Quality: {quality}")

        # 將原始封包轉送給遠端伺服器
        send_sock.sendto(data, (REMOTE_IP, REMOTE_PORT))

    except Exception as e:
        print(f"⚠️ Error decoding data: {data} — {e}")

