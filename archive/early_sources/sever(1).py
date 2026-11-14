# pc_keyboard_udp_server.py  (PC=UDP Server，等待 Jetson HELLO 後開始送)
# -*- coding: utf-8 -*-
import socket, time, keyboard

HOST = '0.0.0.0'
PORT = 5000
UPDATE_INTERVAL = 0.011  # ~90Hz

# 你的原本界限
BLDC_MIN, BLDC_STOP, BLDC_MAX = 145, 152, 160
STEER_MIN, STEER_CTR, STEER_MAX = 94, 152, 210
SPEED_STEP, TURN_STEP = 1, 1

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, PORT))
    sock.settimeout(0.5)

    print(f"[PC/UDP] Server listening on {HOST}:{PORT} … 等待 Jetson HELLO")
    jetson_addr = None

    # 等待 Jetson 先送 'HELLO'
    while jetson_addr is None:
        try:
            data, addr = sock.recvfrom(64)
            if data == b'HELLO':
                jetson_addr = addr
                print(f"[PC/UDP] Paired with Jetson at {jetson_addr}")
        except socket.timeout:
            continue

    speed, turn = BLDC_STOP, STEER_CTR
    last = time.time()
    try:
        while True:
            now = time.time()
            if now - last < UPDATE_INTERVAL:
                time.sleep(0.001); continue
            last = now

            # 急剎車
            if keyboard.is_pressed('space'):
                speed, turn = BLDC_STOP, STEER_CTR

            # 前後
            if keyboard.is_pressed('w'):
                speed = clamp(speed + SPEED_STEP, BLDC_MIN, BLDC_MAX)
            elif keyboard.is_pressed('s'):
                speed = clamp(speed - SPEED_STEP, BLDC_MIN, BLDC_MAX)
            else:
                if   speed > BLDC_STOP: speed -= SPEED_STEP
                elif speed < BLDC_STOP: speed += SPEED_STEP

            # 左右
            if keyboard.is_pressed('a'):
                turn = clamp(turn - TURN_STEP, STEER_MIN, STEER_MAX)
            elif keyboard.is_pressed('d'):
                turn = clamp(turn + TURN_STEP, STEER_MIN, STEER_MAX)
            else:
                if   turn < STEER_CTR: turn += 1
                elif turn > STEER_CTR: turn -= 1

            pkt = bytes([speed & 0xFF, turn & 0xFF])
            sock.sendto(pkt, jetson_addr)
            print(f"[PC/UDP] -> {jetson_addr}  speed={speed:3d} turn={turn:3d}")
    except KeyboardInterrupt:
        print("\n[PC/UDP] bye")
    finally:
        sock.close()

if __name__ == "__main__":
    main()
