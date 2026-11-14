# jetson_motor_uart_verbose_udp_client.py (Jetson=UDP Client，對 PC 發 HELLO，之後接收並送 UART)
# -*- coding: utf-8 -*-
import socket
import serial
from itertools import count
from datetime import datetime
import time

PC_HOST, PC_PORT = '120.126.10.41', 5000   # ← PC 的位址與 Server 監聽的埠
SERDEV = '/dev/ttyTHS0'
BAUD   = 115200
LOG_EVERY = 1
HELLO_INTERVAL = 2.0   # 每 2 秒打一次招呼，避免 PC 端重啟配對丟失

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

def main():
    # UART
    ser = serial.Serial(
        SERDEV, BAUD,
        bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
        timeout=0, write_timeout=0, rtscts=False, dsrdtr=False, xonxoff=False
    )
    ser.reset_output_buffer(); ser.reset_input_buffer()

    # UDP（不需 bind，Client 直接送 HELLO 並接收）
    us = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    us.settimeout(0.5)

    print(f"[JETSON] UDP client → {PC_HOST}:{PC_PORT}  |  UART {SERDEV}@{BAUD}")
    last_hello = 0.0

    seq_gen = count(0)
    packets = 0
    try:
        while True:
            # 定期打 HELLO（pc 重啟時讓它重新配對）
            now = time.time()
            if now - last_hello > HELLO_INTERVAL:
                us.sendto(b'HELLO', (PC_HOST, PC_PORT))
                last_hello = now

            try:
                data, addr = us.recvfrom(64)
            except socket.timeout:
                continue

            if len(data) < 2:
                continue

            speed, turn = data[0], data[1]
            seq = next(seq_gen)
            pkt = build_uart(speed, turn, seq)
            n = ser.write(pkt)

            packets += 1
            if LOG_EVERY and packets % LOG_EVERY == 0:
                ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                print(f"[{ts}] RX_UDP {addr[0]}:{addr[1]}  speed={speed:3d}  turn={turn:3d}  |  "
                      f"TX_UART seq={seq:5d} bytes={n} pkt=[{hexstr(pkt)}]", flush=True)

    except KeyboardInterrupt:
        print("\n[JETSON] bye")
    finally:
        try: us.close()
        except: pass
        try: ser.close()
        except: pass

if __name__ == "__main__":
    main()

