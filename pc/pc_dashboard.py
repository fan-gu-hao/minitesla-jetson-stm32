#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MiniTesla 統一操作視窗。沿用 pc_control.py 的唯一控制傳送路徑。

影像／雷達接收與解碼在 pc_media.py 的子程序執行；Tk 主執行緒負責顯示，
完成可見影格的顯示後才回覆確認。Python 3.8 以上，無新增第三方套件。
"""

import argparse
from collections import deque
import importlib.util
import math
import multiprocessing
import queue
import socket
import sys
import time

from pc_control import ControlClientState, ControlWindow, SHORTCUT_ACTIONS


VIDEO_WIDTH, VIDEO_HEIGHT = 640, 360
LIDAR_STALE_S = 0.5


class DashboardWindow(ControlWindow):
    """共用既有控制狀態機與唯一 UDP 傳送器，不新增控制執行緒。"""

    def __init__(self, root, sock, bind, port, video_port, lidar_port,
                 video_events, lidar_events, display_acks, media_stop, media_process):
        import tkinter as tk
        from tkinter import ttk

        self.root, self.sock = root, sock
        self.state = ControlClientState()
        self.closing = False
        self.shortcut_held = set()
        self._notified_offers = deque(maxlen=32)
        self._blocked_shortcuts = set()
        self.last_render = 0.0
        self.network_error = ""
        self.video_events, self.lidar_events = video_events, lidar_events
        self.display_acks, self.media_stop = display_acks, media_stop
        self.media_process = media_process
        self._photo_factory = tk.PhotoImage
        self._photo = None
        self._frame_key = None
        self._frame_deadline = 0.0
        self._video_connection = None
        self._last_video_id = -1
        self._lidar_points = []
        self._lidar_last_packet = None
        self._last_lidar_draw = 0.0
        self._outside_focus = False
        self._was_visible = True
        self._fatal_message = ""

        root.title("MiniTesla｜統一操作台")
        root.geometry("1280x850")
        root.minsize(1120, 800)
        outer = ttk.Frame(root, padding=12)
        outer.pack(fill="both", expand=True)
        ttk.Label(outer, text="MiniTesla 室內智慧車", font=("Microsoft JhengHei", 17, "bold")).pack(anchor="w")
        ttk.Label(outer, text="影像、雷達與控制共用此視窗；切換到其他程式會要求停止待命。").pack(anchor="w", pady=(3, 8))
        # 邀請固定在頂部，不會因捲動右側狀態欄而消失，也不彈出搶焦點視窗。
        mission_group = ttk.LabelFrame(outer, text="巡邏找位與停車決定", padding=6)
        mission_group.pack(fill="x", pady=(0, 8))
        self.mission_text = tk.StringVar()
        ttk.Label(mission_group, textvariable=self.mission_text, wraplength=740,
                  justify="left").pack(side="left", fill="x", expand=True)
        self.accept_button = ttk.Button(mission_group, text="Y　同意入庫", takefocus=False,
                                        command=lambda: self._action("park_accept"))
        self.decline_button = ttk.Button(mission_group, text="N　略過繼續找位", takefocus=False,
                                         command=lambda: self._action("park_decline"))
        self.accept_button.pack(side="left", padx=4)
        self.decline_button.pack(side="left", padx=4)
        body = ttk.Frame(outer)
        body.pack(fill="both", expand=True)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)
        left = ttk.Frame(body)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 14))
        right_holder = ttk.Frame(body)
        right_holder.grid(row=0, column=1, sticky="nsew")
        right_canvas = tk.Canvas(right_holder, highlightthickness=0, width=440, takefocus=0)
        right_scroll = ttk.Scrollbar(right_holder, orient="vertical", command=right_canvas.yview)
        right_scroll.pack(side="right", fill="y")
        right_canvas.pack(side="left", fill="both", expand=True)
        right_canvas.configure(yscrollcommand=right_scroll.set)
        right = ttk.Frame(right_canvas)
        right_item = right_canvas.create_window((0, 0), window=right, anchor="nw")
        right.bind("<Configure>", lambda _event: right_canvas.configure(scrollregion=right_canvas.bbox("all")))
        right_canvas.bind("<Configure>", lambda event: right_canvas.itemconfigure(right_item, width=event.width))

        def scroll_status(event):
            widget = event.widget
            while widget is not None and widget is not right_holder:
                widget = getattr(widget, "master", None)
            if widget is right_holder:
                delta = -1 if getattr(event, "num", None) == 4 or getattr(event, "delta", 0) > 0 else 1
                right_canvas.yview_scroll(delta, "units")
                return "break"

        root.bind("<MouseWheel>", scroll_status)
        root.bind("<Button-4>", scroll_status)
        root.bind("<Button-5>", scroll_status)

        image_group = ttk.LabelFrame(left, text="車端影像", padding=4)
        image_group.pack(fill="x")
        self.video_canvas = tk.Canvas(image_group, width=VIDEO_WIDTH, height=VIDEO_HEIGHT,
                                      bg="#101820", highlightthickness=0, takefocus=0)
        self.video_canvas.pack()
        self._image_item = self.video_canvas.create_image(VIDEO_WIDTH // 2, VIDEO_HEIGHT // 2)
        self._placeholder_item = self.video_canvas.create_text(VIDEO_WIDTH // 2, VIDEO_HEIGHT // 2,
            text="等待車端影像", fill="#d9e2ea", font=("Microsoft JhengHei", 14))
        self.media_text = tk.StringVar(value="影像接收：等待連線")
        ttk.Label(image_group, textvariable=self.media_text, wraplength=630).pack(anchor="w", pady=(4, 0))

        lidar_group = ttk.LabelFrame(left, text="雷達原始掃描", padding=4)
        lidar_group.pack(fill="both", expand=True, pady=(8, 0))
        self.lidar_canvas = tk.Canvas(lidar_group, width=VIDEO_WIDTH, height=245,
                                      bg="#101820", highlightthickness=0, takefocus=0)
        self.lidar_canvas.pack(fill="both", expand=True)
        self.lidar_text = tk.StringVar(value="雷達：等待資料")
        ttk.Label(lidar_group, textvariable=self.lidar_text).pack(anchor="w", pady=(3, 0))
        ttk.Label(lidar_group, text="此圖為原始雷達角度；車端依校正資料定位入庫，四向測距提供保護。",
                  wraplength=630).pack(anchor="w")

        for name in ("status", "output", "reason", "action", "health", "video", "patrol", "parking", "perception"):
            setattr(self, name + "_text", tk.StringVar())
        ttk.Label(right, textvariable=self.status_text, wraplength=420,
                  font=("Microsoft JhengHei", 12, "bold")).pack(anchor="w", fill="x")
        buttons = ttk.Frame(right)
        buttons.pack(fill="x", pady=(9, 7))
        for col in range(2):
            buttons.columnconfigure(col, weight=1)
        self.manual_button = ttk.Button(buttons, text="M　手動接管", takefocus=False,
                                         command=lambda: self._action("manual"))
        self.patrol_button = ttk.Button(buttons, text="P　反應式巡邏", takefocus=False,
                                         command=lambda: self._action("patrol"))
        self.park_button = ttk.Button(buttons, text="K　自動找位入庫", takefocus=False,
                                       command=lambda: self._action("park"))
        self.reset_button = ttk.Button(buttons, text="R　解除故障", takefocus=False,
                                        command=lambda: self._action("reset"))
        self.search_button = ttk.Button(buttons, text="G　巡邏找位", takefocus=False,
                                         command=lambda: self._action("search"))
        for widget, row, col in ((self.manual_button, 0, 0), (self.patrol_button, 0, 1),
                                  (self.park_button, 1, 0), (self.reset_button, 1, 1)):
            widget.grid(row=row, column=col, sticky="ew", padx=2, pady=3)
        self.search_button.grid(row=2, column=0, columnspan=2, sticky="ew", padx=2, pady=3)
        ttk.Button(buttons, text="空白鍵　停止待命", takefocus=False,
                   command=lambda: self._action("stop")).grid(row=3, column=0, columnspan=2, sticky="ew", padx=2, pady=4)
        ttk.Label(right, text="W／S 前進／倒退；A／D 轉向。\n"
                  "切換模式後需重新按鍵；自主模式時 WASD 不接管。\n"
                  "待命時 G 巡邏找位；找到後停車通知，再按 Y 同意或 N 略過。\n"
                  "K 保留直接找可見車位入庫；P 保留普通巡邏。\n"
                  "車位座標以後壁中央為原點，橫向向左為正。",
                  wraplength=420).pack(anchor="w", pady=(0, 7))
        for name in ("action", "reason", "output", "health", "video", "patrol", "parking", "perception"):
            ttk.Separator(right, orient="horizontal").pack(fill="x", pady=(4, 3))
            ttk.Label(right, textvariable=getattr(self, name + "_text"),
                      wraplength=420, justify="left").pack(anchor="w", fill="x")
        ttk.Label(right, text="本機監聽 {}\n控制 {}／雷達 {}（UDP）／影像 {}（TCP）".format(
            bind, port, lidar_port, video_port), wraplength=420).pack(anchor="w", pady=(8, 0))

        root.bind("<KeyPress>", self._key_down)
        root.bind("<KeyRelease>", self._key_up)
        root.bind("<FocusOut>", self._focus_out)
        root.protocol("WM_DELETE_WINDOW", self._close)
        self.video_canvas.bind("<Button-1>", lambda _event: root.focus_set())
        self.lidar_canvas.bind("<Button-1>", lambda _event: root.focus_set())
        root.after_idle(root.focus_set)
        self._render()
        root.after(0, self._poll)
        root.after(20, self._media_tick)

    def _action(self, name):
        if not self.closing:
            return super()._action(name)

    def _key_down(self, event):
        if self.closing:
            return "break"
        key = event.keysym.lower()
        if key in self._blocked_shortcuts:
            return "break"
        return super()._key_down(event)

    def _key_up(self, event):
        self._blocked_shortcuts.discard(event.keysym.lower())
        return super()._key_up(event)

    def _check_focus(self):
        if self.closing:
            return
        outside = self.root.focus_get() is None
        if outside:
            # Alt+Tab 可能不送來 KeyRelease：模式鍵須先放開，防止返回視窗時
            # 系統自動重複按鍵重新啟動巡邏／入庫。空白鍵始終可再次要求停止。
            self._blocked_shortcuts.update(self.shortcut_held.intersection(SHORTCUT_ACTIONS))
            self.shortcut_held.clear()
            self.state.stop_held = False
            if not self._outside_focus:
                self.state.request_action("stop")
            self.state.clear_inputs()
        self._outside_focus = outside

    def _render(self):
        super()._render()
        # 第六階段相容層：斷線時不能把最後收到的測距值標成即時狀態。
        if not self.state.connected:
            self.health_text.set("M4：未知（車端狀態中斷）\n四向距離：前／後／左／右均未知")
            self.output_text.set("未連線：不送出行駛命令；車端目前實際輸出未知。")
            self.video_text.set("車端影格確認：未知（狀態中斷）")
        else:
            distances = self.state.health.get("tof_mm")
            distances = distances if isinstance(distances, dict) else {}
            labels = []
            for key, label in (("front", "前"), ("rear", "後"), ("left", "左"), ("right", "右")):
                value = distances.get(key)
                valid = isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value >= 0
                labels.append("{}：{}".format(label, "{:.0f} 毫米".format(value) if valid else "未知"))
            self.health_text.set("M4：{}\n{}\n{}".format(
                "已連線" if self.state.health.get("m4_connected") else "未連線",
                "　".join(labels[:2]), "　".join(labels[2:])))

    def _check_visibility(self):
        visible = self._visible()
        if not visible and self._was_visible and not self.closing:
            self._blocked_shortcuts.update(self.shortcut_held.intersection(SHORTCUT_ACTIONS))
            self.shortcut_held.clear()
            self.state.stop_held = False
            self.state.request_action("stop")
            self.state.clear_inputs()
        self._was_visible = visible

    def _clear_video(self, message):
        self._frame_key = None
        self._frame_deadline = 0.0
        self._photo = None
        self.video_canvas.itemconfigure(self._image_item, image="")
        self.video_canvas.itemconfigure(self._placeholder_item, text=message, state="normal")

    def _visible(self):
        try:
            return bool(self.root.winfo_viewable() and self.video_canvas.winfo_viewable())
        except Exception:
            return False

    def _confirm_display(self, key, deadline):
        # 此回呼在 Tk 完成本輪視窗工作後執行。禁止在背景解碼時自行確認。
        if (self.closing or key != self._frame_key or key[0] != self._video_connection
                or time.monotonic() >= deadline or not self._visible()):
            return
        try:
            self.display_acks.put_nowait({"connection_id": key[0], "frame_id": key[1]})
            self.media_text.set("影像：已顯示影格 {}".format(key[1]))
        except queue.Full:
            # 主程序或接收程序卡住時讓車端確認逾時，不延長過期影格。
            pass

    def _video_event(self, event):
        kind = event.get("kind")
        if kind == "video_state":
            state, connection = event.get("state"), event.get("connection_id")
            if state == "error":
                self._fatal(str(event.get("message") or "影像接收程序錯誤"))
                return
            if state == "connected":
                self._video_connection = connection
                self._last_video_id = -1
                self._clear_video("已連線，等待新影格")
            elif state == "disconnected":
                if connection != self._video_connection:
                    return
                self._video_connection = None
                self._clear_video("影像連線中斷")
            elif state == "listening" and self._video_connection is None:
                self._clear_video("等待車端影像")
            self.media_text.set(str(event.get("message") or "影像：等待資料"))
            return
        if kind != "video_frame" or self.closing:
            return
        connection, frame_id = event.get("connection_id"), event.get("frame_id")
        deadline = event.get("deadline")
        if (connection is None or connection != self._video_connection
                or not isinstance(frame_id, int) or isinstance(frame_id, bool)
                or frame_id <= self._last_video_id or not isinstance(deadline, (int, float))
                or not math.isfinite(deadline) or time.monotonic() >= deadline):
            return
        self._last_video_id = frame_id
        if not self._visible():
            self._clear_video("視窗未顯示，暫停影格確認")
            return
        try:
            photo = self._photo_factory(data=event["ppm"], format="PPM")
        except Exception as exc:
            self._fatal("無法顯示影格：" + str(exc))
            return
        if self.closing or time.monotonic() >= deadline or not self._visible():
            return
        self._photo = photo  # Tk 不保存 Python 圖片參考，須保留至下張影格。
        self.video_canvas.itemconfigure(self._image_item, image=photo)
        self.video_canvas.itemconfigure(self._placeholder_item, state="hidden")
        key = (connection, frame_id)
        self._frame_key, self._frame_deadline = key, deadline
        self.media_text.set("影像：已顯示新影格 {}；等待畫面確認".format(frame_id))
        self.root.after_idle(lambda: self._confirm_display(key, deadline))

    def _lidar_event(self, event):
        if event.get("kind") == "lidar_state" and event.get("state") == "error":
            self._fatal(str(event.get("message") or "雷達接收程序錯誤"))
        elif event.get("kind") == "lidar_scan":
            self._lidar_points = event.get("points", [])[:360]
            self._lidar_last_packet = event.get("last_packet_at")

    def _draw_lidar(self, now):
        canvas = self.lidar_canvas
        width, height = max(canvas.winfo_width(), 200), max(canvas.winfo_height(), 180)
        cx, cy = width / 2.0, height / 2.0
        radius = max(30.0, min(width, height) / 2.0 - 24.0)
        canvas.delete("all")
        for mm in (2000, 4000, 6000):
            r = radius * mm / 6000.0
            canvas.create_oval(cx - r, cy - r, cx + r, cy + r, outline="#304451")
        canvas.create_line(cx - radius, cy, cx + radius, cy, fill="#526672")
        canvas.create_line(cx, cy - radius, cx, cy + radius, fill="#526672")
        for x, y, label in ((cx + radius + 13, cy, "0°"), (cx, cy - radius - 11, "90°"),
                            (cx - radius - 16, cy, "180°"), (cx, cy + radius + 12, "270°")):
            canvas.create_text(x, y, text=label, fill="#b9c9d2", font=("Arial", 9))
        valid_points = []
        for point in self._lidar_points:
            try:
                angle, distance, at = point
                if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                           for v in (angle, distance, at)):
                    continue
                if not 0 <= now - at <= LIDAR_STALE_S or not 0 < distance <= 6000:
                    continue
            except (ValueError, TypeError):
                continue
            valid_points.append(point)
            theta, rr = math.radians(angle), radius * distance / 6000.0
            x, y = cx + rr * math.cos(theta), cy - rr * math.sin(theta)
            canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill="#52d7ba", outline="")
        canvas.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill="#f5cf65", outline="")
        self._lidar_points = valid_points
        self.lidar_text.set("雷達：{}；顯示半徑 6 公尺（每圈 2 公尺）".format(
            "有效點 {}，逾 0.5 秒即移除".format(len(valid_points)) if valid_points else "尚無有效資料／資料逾時"))

    def _media_tick(self):
        if self.closing:
            return
        for events, handler in ((self.video_events, self._video_event), (self.lidar_events, self._lidar_event)):
            for _ in range(8):
                try:
                    event = events.get_nowait()
                except queue.Empty:
                    break
                if isinstance(event, dict):
                    handler(event)
                if self.closing:
                    return
        if self.media_stop.is_set() or not self.media_process.is_alive():
            self._fatal("影像／雷達接收程序已停止；已要求車輛停止，請查看終端機訊息。")
            return
        now = time.monotonic()
        self._check_visibility()
        if self._frame_key is not None and (now >= self._frame_deadline or not self._visible()):
            self._clear_video("影像逾時／視窗未顯示，等待新影格")
            self.media_text.set("影像：沒有可用的即時畫面")
        if now - self._last_lidar_draw >= 0.1:
            self._draw_lidar(now)
            self._last_lidar_draw = now
        self._check_focus()
        self.root.after(20, self._media_tick)

    def _fatal(self, message):
        if self.closing:
            return
        self._fatal_message = message
        print("操作台停止：" + message, file=sys.stderr, flush=True)
        self.network_error = message
        self._close()

    def _close_send(self, attempt):
        self._send()
        if attempt < 2:
            self.root.after(40, lambda: self._close_send(attempt + 1))
        else:
            self.media_stop.set()
            self.sock.close()
            self.root.destroy()


def finish_media(process, stop, queues):
    """離開視窗迴圈後才等待子程序，避免停止操作被清理工作阻塞。"""
    stop.set()
    if process is not None and process.pid is not None:
        process.join(timeout=2.0)
        if process.is_alive():
            process.terminate()
            process.join(timeout=1.0)
        if process.is_alive():
            process.kill()
            process.join(timeout=1.0)
    for channel in queues:
        channel.cancel_join_thread()
        channel.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description="MiniTesla 統一影像、雷達與車輛控制操作台")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5000, help="控制 UDP 連接埠")
    parser.add_argument("--video-port", type=int, default=7000, help="影像 TCP 連接埠")
    parser.add_argument("--lidar-port", type=int, default=6000, help="雷達 UDP 連接埠")
    args = parser.parse_args(argv)
    if any(not 1 <= port <= 65535 for port in (args.port, args.video_port, args.lidar_port)):
        parser.error("連接埠必須介於 1～65535")
    if args.port == args.lidar_port:
        parser.error("控制與雷達 UDP 連接埠不能相同")
    missing = [name for name in ("cv2", "numpy") if importlib.util.find_spec(name) is None]
    if missing:
        raise SystemExit("目前 Python 環境缺少影像接收套件：{}。請使用原先能執行 pc_receive.py 的環境。".format(
            "、".join(missing)))
    try:
        import tkinter as tk
        from pc_media import media_worker
    except ImportError as exc:
        raise SystemExit("無法載入視窗／接收模組：{}。請保留同層的 pc_control.py、pc_receive.py 與 pc_media.py。".format(exc))
    context = multiprocessing.get_context("spawn")
    stop = context.Event()
    channels = [context.Queue(maxsize=count) for count in (4, 2, 4)]
    process, window, root = None, None, None
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        try:
            if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            sock.bind((args.bind, args.port))
            sock.setblocking(False)
        except OSError as exc:
            raise SystemExit("無法監聽控制 UDP {}：{}。請先關閉舊 pc_control.py／sever(1).py 與其他操作台。".format(args.port, exc))
        try:
            root = tk.Tk()
        except tk.TclError as exc:
            raise SystemExit("無法開啟操作視窗：" + str(exc))
        process = context.Process(target=media_worker,
            args=(args.bind, args.video_port, args.lidar_port, *channels, stop), name="MiniTesla-media")
        process.start()
        window = DashboardWindow(root, sock, args.bind, args.port, args.video_port, args.lidar_port,
                                 *channels, stop, process)
        print("MiniTesla 統一操作台：M 手動、P 巡邏、K 入庫、G 巡邏找位、Y 同意、N 略過、空白鍵停止；關閉視窗即結束全部接收程序。", flush=True)
        root.mainloop()
        return 1 if window._fatal_message else 0
    except KeyboardInterrupt:
        return 130
    finally:
        if window is not None and not window.closing:
            window.closing = True
            window.shortcut_held.clear()
            window.state.request_action("stop")
            for _ in range(3):
                window._send()
        sock.close()
        finish_media(process, stop, channels)
        if root is not None:
            try:
                root.destroy()
            except tk.TclError:
                pass


if __name__ == "__main__":
    multiprocessing.freeze_support()
    raise SystemExit(main())
