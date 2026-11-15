#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""智慧車電腦端控制視窗（第 10 階段）。只使用 Python 標準函式庫。

取代舊的 sever(1).py；請搭配本階段的 pc_receive.py 影像／雷達接收視窗。
Jetson 主動送出狀態，電腦回覆同一個 UDP 來源位址，沿用原 NAT 路徑。
此視窗必須有焦點才能用 WASD 操作；離開視窗即要求停止待命。
"""

import argparse
from collections import deque
import json
import math
import re
import socket
import time


ESC_MIN, ESC_STOP, ESC_MAX = 145, 152, 160
SERVO_MIN, SERVO_CENTER, SERVO_MAX = 94, 152, 210
STATUS_TIMEOUT = 0.5
SEND_INTERVAL_MS = 11
SESSION_RE = re.compile(r"^[0-9a-fA-F]{32}$")
MODE_LABELS = {"standby": "待命", "manual": "手動", "patrol": "反應式巡邏", "park": "自動找位入庫", "fault": "故障待命",
               "search": "巡邏找位", "offer": "停止確認車位"}
PATROL_LABELS = {"idle": "未啟動", "handoff": "中立交接", "cruise": "向前巡行",
                 "turn_left": "向左繞行", "turn_right": "向右繞行", "blocked": "停止等待空間",
                 "stopped": "已停止，清除障礙後需重新按 P"}
PARKING_LABELS = {"idle": "未啟動", "handoff": "中立交接", "preparing": "中立準備中",
                  "searching": "掃描目前可見範圍，尋找車位", "acquiring": "確認車位位置穩定",
                  "planning": "停止並規劃入庫路徑", "aligning": "向前調整入庫位置",
                  "reversing": "倒車入庫中", "gear_wait": "中立等待換向",
                  "settling": "停止並確認停妥",
                  "complete": "入庫完成", "aborted": "已停止，需重新操作"}
ACTION_LABELS = {"manual": "手動接管", "patrol": "反應式巡邏", "park": "自動找位入庫", "stop": "停止待命", "reset": "解除故障",
                 "search": "巡邏找位", "park_accept": "同意入庫", "park_decline": "略過並繼續找位"}
MISSION_LABELS = {"idle": "未啟動", "searching": "巡邏並尋找車位", "verifying": "已停止，確認同一車位穩定",
                  "awaiting": "等待本次停車決定", "skipping": "已略過，繼續找位",
                  "parking": "已同意，正在入庫", "completed": "任務完成", "aborted": "任務已停止"}
SHORTCUT_ACTIONS = {"m": "manual", "p": "patrol", "k": "park", "r": "reset",
                    "g": "search", "y": "park_accept", "n": "park_decline"}
REASON_LABELS = {
    "search_selected": "已選擇巡邏找位",
    "search_input_timeout": "巡邏找位監督心跳逾時",
    "offer_input_timeout": "停車決定監督心跳逾時",
    "mission_searching": "巡邏中，尋找雷達可辨識的車位",
    "mission_candidate_found": "發現候選車位，先停止確認",
    "mission_verifying": "保持中立，確認同一車位穩定",
    "mission_awaiting_decision": "車輛已停止，等待你的停車決定",
    "mission_declined": "已略過本次車位，繼續找位",
    "mission_accepted": "已同意停入本次車位",
    "mission_search_timeout": "找位任務時間已到，停止待命",
    "mission_decision_timeout": "停車決定時間已到，停止待命",
    "mission_verify_timeout": "車位穩定確認逾時，停止待命",
    "mission_worker_timeout": "找位程序回報逾時，停止待命",
    "mission_source_stale": "找位雷達來源資料逾時，停止待命",
    "mission_target_lost": "無法持續辨認本次車位，停止待命",
    "mission_report_invalid": "找位程序資料不合規，停止待命",
    "mission_not_stationary": "車輛尚未穩定停止，不能確認停車",
    "mission_no_offer": "目前沒有有效的停車通知",
    "mission_offer_mismatch": "停車通知已變更，忽略舊決定",
    "mission_requires_standby": "請先按空白鍵停止，確認待命後再按 G",
    "mission_m4_reboot": "M4 已重新啟動，停止找位任務",
    "mission_path_blocked": "找位行駛空間不足，停止待命",
    "mission_found": "發現候選車位，先停止確認",
    "mission_ready": "車位確認完成，等待你的停車決定",
    "mission_skipping": "已略過本車位，繼續巡邏找位",
    "mission_invalid": "找位任務資料無效，停止待命",
    "mission_target_missing": "缺少本次車位資料，停止待命",
    "mission_acquire_timeout": "車位穩定確認逾時，停止待命",
    "mission_sensor_invalid": "找位感測資料無效，停止待命",
    "mission_vehicle_moved": "等待決定期間車輛位置改變，停止待命",
    "mission_lidar_unavailable": "雷達資料無效或逾時，停止找位任務",
    "mission_pose_lost": "無法持續定位同一車位，停止找位任務",
    "mission_speed_exceeded": "估計車輛移動過快，停止找位任務",
    "mission_pose_jump": "車位定位突然改變，停止找位任務",
    "park_approved_target_missing": "已同意的車位資料缺失，停止入庫",
    "video_unconfirmed": "尚未收到電腦端影格確認，請先啟動新版影像接收器",
    "video_disconnected": "影像連線中斷；恢復後按 R，再選 M、P 或 K",
    "video_ack_timeout": "影格確認逾時；恢復後按 R，再選 M、P 或 K",
    "video_frame_stale": "影格已過期，請檢查攝影機、推論及網路",
    "video_sender_stopped": "影像傳送程序已停止，請重新啟動車端主程式",
    "startup": "開機待命", "operator_stop": "操作者要求停止", "stopped": "已停止",
    "manual_selected": "已選擇手動控制", "fault_reset": "故障已解除，回到待命",
    "m4_offline": "M4 通訊中斷", "m4_guard_latched": "M4 保護尚未解除",
    "m4_state_unknown": "M4 狀態無法確認", "manual_input_timeout": "手動控制心跳逾時",
    "reset_required": "請先解除故障，再選擇 M 手動、P 巡邏或 K 入庫", "neutral_required": "切換前須回到中立",
    "stop_before_reset": "請先停止，再解除故障", "epoch_mismatch": "模式已變更，請重新操作",
    "action_id_conflict": "命令編號衝突，請重新操作", "stale_action": "忽略過期模式命令",
    "feature_unavailable": "這項功能尚未開放", "unknown_action": "無法辨識模式命令",
    "patrol_selected": "已選擇反應式巡邏", "patrol_input_timeout": "巡邏監督心跳逾時",
    "patrol_camera_unavailable": "巡邏影像辨識資料無效或逾時",
    "patrol_path_blocked": "巡邏路徑受阻，請清除障礙後重新選擇模式",
    "patrol_turn_timeout": "繞行逾時，請清除障礙後重新選擇模式",
    "patrol_sensor_invalid": "巡邏測距資料無效或逾時", "patrol_wait_clear": "等待通行空間",
    "patrol_ready": "巡邏已就緒", "patrol_cruise": "前方空間充足，向前巡行",
    "patrol_turn_left": "向左繞行", "patrol_turn_right": "向右繞行",
    "patrol_clock_invalid": "巡邏時鐘資料異常",
    "m4_transport_error": "M4 串列通訊錯誤，請排除原因後重啟車端主程式",
    "park_selected": "已選擇入庫模式", "park_input_timeout": "入庫監督心跳逾時",
    "park_camera_unavailable": "入庫影像辨識資料無效或逾時",
    "park_requires_standby": "請先按空白鍵停止，確認待命後再按 K",
    "park_ready": "入庫已就緒", "park_preparing": "保持中立並確認測距",
    "park_reversing": "倒車入庫中", "park_settling": "停止並確認停妥",
    "park_complete": "已完成入庫", "park_sensor_invalid": "入庫測距資料無效或逾時",
    "park_clock_invalid": "入庫時鐘資料異常", "park_m4_reboot": "M4 已重新啟動，停止入庫",
    "park_sample_regression": "入庫測距樣本時序倒退", "park_sample_invalid": "尚未取得新的入庫測距樣本",
    "park_start_out_of_range": "後方起始距離不在允許範圍", "park_path_blocked": "入庫路徑空間不足",
    "park_prepare_timeout": "中立準備逾時", "park_no_progress": "入庫路段沒有進展，已停止",
    "park_wrong_direction": "後方距離持續增加，方向異常，已停止", "park_distance_jump": "後方距離異常跳變",
    "park_timeout": "入庫總時間逾時", "park_settle_timeout": "停車位置未能穩定",
    "park_target_out_of_range": "目前位置超出入庫完成範圍",
    "auto_park_not_configured": "尚未完成停車設定",
    "park_lidar_unavailable": "雷達資料無效或逾時，停止入庫",
    "park_pose_lost": "無法持續辨認車位位置，停止入庫",
    "park_pose_jump": "車位定位突然改變，停止入庫",
    "park_ambiguous_bay": "可見車位無法唯一確認，請調整車輛位置",
    "park_no_bay": "目前可見範圍未找到合適車位",
    "park_no_path": "找不到具有足夠空間的入庫路徑",
    "park_worker_stale": "入庫規劃資料逾時，停止入庫",
    "park_gear_timeout": "中立換向確認逾時，停止入庫",
    "park_speed_exceeded": "估計行駛速度超出停車設定，停止入庫",
    "park_calibration_invalid": "停車校正資料無效，請檢查設定",
    "park_tracking_error": "車輛偏離規劃路徑，停止入庫",
    "park_internal_error": "停車程序內部狀態錯誤，停止入庫",
    "auto_park_m4_reboot": "M4 已重新啟動，停止自動入庫",
    "auto_park_path_blocked": "車輛周圍空間不足，停止自動入庫",
    "auto_park_plan_timeout": "入庫路徑規劃逾時，已停止",
    "auto_park_report_invalid": "停車程序回報資料不合規，已停止",
    "auto_park_search_timeout": "尋找可見車位逾時，已停止",
    "auto_park_source_stale": "入庫定位來源資料逾時，已停止",
    "auto_park_state_invalid": "停車程序狀態不一致，已停止",
    "auto_park_timeout": "自動入庫總時間逾時，已停止",
    "auto_park_worker_timeout": "停車規劃程序回報逾時，已停止",
    "auto_park_tof_invalid_front": "前方測距資料無效或逾時，停止自動入庫",
    "auto_park_tof_invalid_rear": "後方測距資料無效或逾時，停止自動入庫",
    "auto_park_tof_invalid_left": "左方測距資料無效或逾時，停止自動入庫",
    "auto_park_tof_invalid_right": "右方測距資料無效或逾時，停止自動入庫",
    "park_searching": "停止掃描目前可見範圍，尋找車位",
    "park_acquiring": "確認車位位置與方向穩定",
    "park_planning": "保持中立，規劃入庫路徑",
    "park_aligning": "沿規劃路徑向前調整位置",
    "park_gear_wait": "等待中立輸出確認後換向",
    "shutdown": "車端程式結束",
}


def describe_reason(reason):
    if not reason:
        return "正常"
    if reason in REASON_LABELS:
        return REASON_LABELS[reason]
    for key, label in (("front", "前"), ("rear", "後"), ("left", "左"), ("right", "右")):
        if reason == "tof_invalid_" + key:
            return label + "方測距資料無效或逾時"
        if reason == "mission_tof_invalid_" + key:
            return label + "方測距資料無效或逾時，停止找位任務"
    return str(reason)


def action_label(name, parking=None):
    if name == "park" and isinstance(parking, dict) and parking.get("range_source") == "rear_tof":
        return "舊版直線入庫"
    return ACTION_LABELS[name]


def _integer(value):
    return isinstance(value, int) and not isinstance(value, bool)


def perception_text(perception, current=True):
    """失聯／失效不能顯示為沒有人；警示以累計值呈現，不逐影格跳視窗。"""
    count = perception.get("person_count")
    events = perception.get("event_count")
    events_text = str(events) if _integer(events) and events >= 0 else "未知"
    if not current or perception.get("valid") is not True or not _integer(count) or count < 0:
        status = "未知（影像辨識未就緒或資料過期）"
    elif perception.get("person_present") is True and count > 0:
        status = "警示：目前偵測到 {} 人；依一般障礙物處理".format(count)
    elif perception.get("person_present") is False and count == 0:
        status = "本影格未辨識到人物"
    else:
        status = "未知（人物資料不一致）"
    return "人物：{}；累計警示 {} 次".format(status, events_text)


def parking_text(parking, current=True):
    """過期狀態不能表示目前已完成；遺漏的停車欄位顯示未知。"""
    if not isinstance(parking, dict):
        parking = {}
    if parking.get("range_source") == "lidar_tof":
        return auto_parking_text(parking, current=current)
    if not current:
        return "入庫：未知（資料過期或連線中斷）；後方／目標距離：未知"
    state = parking.get("state")
    label = PARKING_LABELS.get(state, "未知")
    if state == "complete" and parking.get("completed") is not True:
        label = "未知（完成狀態不一致）"
    def distance(value):
        if isinstance(value, (int, float)) and not isinstance(value, bool) and 0 <= value < float("inf"):
            return "{:.0f} 毫米".format(value)
        return "未知"
    return "入庫：{}；後方／目標距離：{}／{}；原因：{}".format(
        label, distance(parking.get("rear_mm")), distance(parking.get("target_mm")),
        describe_reason(parking.get("reason")))


def auto_parking_text(parking, current=True):
    """顯示車位座標；定位資料失效時不將歷史位置當成目前位置。"""
    if not current:
        return "自動入庫：未知（資料過期或連線中斷）；車位位置與剩餘路程：未知"
    if parking.get("reason") == "auto_park_not_configured":
        return "自動入庫：尚未完成停車設定；K 暫停使用"
    state = parking.get("state")
    label = PARKING_LABELS.get(state, "未知")
    reason = parking.get("reason")
    if state == "complete" and parking.get("completed") is not True:
        label = "未知（完成狀態不一致）"
        reason = "完成旗標缺失或不一致"

    def number(value, angle=False, nonnegative=False):
        if (not isinstance(value, (int, float)) or isinstance(value, bool)
                or not math.isfinite(value) or (nonnegative and value < 0)):
            return "未知"
        if angle:
            return "{:+.1f}°".format(math.degrees(math.atan2(math.sin(value), math.cos(value))))
        return "{:.2f} 公尺".format(value)

    def coordinates(value):
        if isinstance(value, dict):
            value = [value.get("x"), value.get("y"), value.get("yaw")]
        if not isinstance(value, (tuple, list)) or len(value) != 3:
            value = [None, None, None]
        return "縱向 {}／橫向 {}／車頭角 {}".format(
            number(value[0]), number(value[1]), number(value[2], angle=True))

    pose = parking.get("pose")
    if reason in ("park_pose_lost", "park_pose_jump", "park_lidar_unavailable", "park_worker_stale",
                  "auto_park_source_stale", "auto_park_worker_timeout", "auto_park_report_invalid"):
        pose = None
    position_label = "車輛位置" if parking.get("active") is True else "最後定位"
    return "自動入庫：{}；原因：{}\n{}（車位座標）：{}\n目標位置：{}\n剩餘路程：{}；路段進度：{}".format(
        label, describe_reason(reason), position_label, coordinates(pose),
        coordinates(parking.get("target")), number(parking.get("path_remaining_m"), nonnegative=True),
        number(parking.get("progress_m"), nonnegative=True))


def _toward(value, target):
    return value + (1 if value < target else -1 if value > target else 0)


class ControlClientState:
    """不依賴視窗或網路的控制狀態，可直接做離線測試。"""

    def __init__(self):
        self.session = None
        self.retired_sessions = deque(maxlen=16)
        self.address = None
        self.connected = False
        self.last_seen = None
        self.mode = "standby"
        self.manual_ready = False
        self.epoch = 0
        self.remote_last_seq = -1
        self.next_seq = 0
        self.last_action_id = 0
        self.pending = None
        self.held = set()
        self.blocked_keys = set()
        self.stop_held = False
        self.esc, self.servo = ESC_STOP, SERVO_CENTER
        self.reason = "等待 Jetson 連線"
        self.action_result = "尚未發出模式命令"
        self.health = {}
        self.video = {}
        self.patrol = {}
        self.parking = {}
        self.perception = {}
        self.capabilities = {}
        self.mission = {}
        self._offer_key = None
        self._offer_deadline = None
        self._dismissed_offer = None
        self._mission_offer_valid = False

    def _observe_mission(self, now):
        """將車端剩餘秒數換成此電腦的截止時間；同一通知不得延長倒數。"""
        data = self.mission
        self._mission_offer_valid = False
        offer_id, remaining, duration = data.get("offer_id"), data.get("remaining_s"), data.get("decision_s")
        if (self.mode != "offer" or data.get("active") is not True or data.get("state") != "awaiting"
                or not _integer(offer_id) or offer_id != self.epoch
                or any(not isinstance(v, (int, float)) or isinstance(v, bool) or not math.isfinite(v)
                       for v in (remaining, duration))
                or not 0 < duration <= 120 or not 0 <= remaining <= duration):
            return
        self._mission_offer_valid = True
        key = (self.session, self.epoch, offer_id)
        deadline = now + remaining
        if key != self._offer_key:
            self._offer_key, self._offer_deadline = key, deadline
        else:
            self._offer_deadline = min(self._offer_deadline, deadline)

    def offer_remaining(self, now=None):
        """只對目前有效、未處理的邀請回傳剩餘秒數；失聯／舊通知均回 None。"""
        now = time.monotonic() if now is None else now
        key = (self.session, self.epoch, self.mission.get("offer_id"))
        if (not self.connected or self.last_seen is None or not 0 <= now - self.last_seen <= STATUS_TIMEOUT
                or self.mode != "offer" or self.capabilities.get("search") is not True
                or self.mission.get("active") is not True or self.mission.get("state") != "awaiting"
                or not self._mission_offer_valid
                or key != self._offer_key or key == self._dismissed_offer or self._offer_deadline is None
                or self.pending is not None or self.stop_held):
            return None
        return max(0.0, self._offer_deadline - now)

    def mission_status(self, now=None):
        now = time.monotonic() if now is None else now
        if not self.connected or self.last_seen is None or not 0 <= now - self.last_seen <= STATUS_TIMEOUT:
            return "找位任務：未知（資料過期或連線中斷）；目前沒有可回覆的停車通知"
        if self.capabilities.get("search") is not True:
            return "找位任務：車端尚未開放；G 暫停使用"
        remaining = self.offer_remaining(now)
        if remaining is not None and remaining > 0:
            return "已找到車位，車輛已停止！剩餘 {} 秒：Y 同意入庫／N 略過繼續找位。\n時間到未回覆：停止待命，不會自行入庫。".format(math.ceil(remaining))
        if self.mode == "offer" and self.mission.get("state") == "awaiting":
            if self.pending and self.pending["name"] in ("park_accept", "park_decline"):
                return "找位任務：已送出本次決定，等待車端確認；車端確認前保持停止。"
            return "找位任務：本次通知已到期或已取消；等待車端狀態，目前不能回覆。"
        return "找位任務：{}；原因：{}".format(
            MISSION_LABELS.get(self.mission.get("state"), "尚無任務資料"), describe_reason(self.mission.get("reason")))

    def clear_inputs(self):
        self.blocked_keys.update(self.held)
        self.held.clear()
        self.esc, self.servo = ESC_STOP, SERVO_CENTER

    def check_timeout(self, now):
        if (self.connected and self.last_seen is not None
                and now - self.last_seen > STATUS_TIMEOUT):
            self.connected = False
            self.manual_ready = False
            self.clear_inputs()
            self.pending = None
            self._dismissed_offer = self._offer_key
            self.reason = "狀態逾時；已清除操作，重連後先要求停止"
            return True
        return False

    def observe(self, message, address, now):
        """接收有效 hello/status；過期模式或舊會話資料不能重啟操作。"""
        if (not isinstance(message, dict) or message.get("v") != 1
                or isinstance(message.get("v"), bool)):
            return False
        if message.get("type") not in ("hello", "status"):
            return False
        token, control = message.get("session"), message.get("control")
        if not isinstance(token, str) or not SESSION_RE.fullmatch(token):
            return False
        if not isinstance(control, dict):
            return False
        mode = control.get("mode")
        epoch, last_seq = control.get("epoch"), control.get("last_seq")
        if (mode not in MODE_LABELS or not _integer(epoch) or epoch < 0
                or not _integer(last_seq) or last_seq < -1):
            return False
        new_session = token != self.session
        if new_session and token in self.retired_sessions:
            return False
        # 已配對後，只接受 hello 建立新的車端會話；舊 status 不得搶回連線。
        if new_session and self.session is not None and message["type"] != "hello":
            return False
        if not new_session:
            if epoch < self.epoch or last_seq < self.remote_last_seq:
                return False
        self.check_timeout(now)
        reconnect = not self.connected or new_session
        if new_session:
            if self.session is not None:
                self.retired_sessions.append(self.session)
            self.clear_inputs()
            self.pending = None
            self.remote_last_seq = -1
        epoch_changed = new_session or epoch != self.epoch
        self.session, self.address = token, address
        self.connected, self.last_seen = True, now
        self.mode, self.epoch, self.remote_last_seq = mode, epoch, last_seq
        self.manual_ready = control.get("manual_ready") is True
        self.next_seq = max(self.next_seq, last_seq + 1)
        self.last_action_id = max(self.last_action_id, last_seq)
        self.reason = str(control.get("reason", ""))
        self.health = message.get("health") if isinstance(message.get("health"), dict) else {}
        self.video = control.get("video") if isinstance(control.get("video"), dict) else {}
        self.patrol = control.get("patrol") if isinstance(control.get("patrol"), dict) else {}
        self.parking = control.get("parking") if isinstance(control.get("parking"), dict) else {}
        self.mission = dict(control["mission"]) if isinstance(control.get("mission"), dict) else {}
        self.perception = control.get("perception") if isinstance(control.get("perception"), dict) else {}
        caps = control.get("capabilities")
        self.capabilities = caps if isinstance(caps, dict) else {}
        self._observe_mission(now)
        ack = control.get("last_action")
        if isinstance(ack, dict) and _integer(ack.get("id")):
            self.last_action_id = max(self.last_action_id, ack["id"])
        if (self.pending is not None and isinstance(ack, dict)
                and ack.get("id") == self.pending["id"]
                and ack.get("name") == self.pending["name"]):
            accepted = ack.get("accepted") is True
            self.action_result = "{}：{}{}".format(
                action_label(self.pending["name"], self.parking), "已接受" if accepted else "未接受",
                "；" + describe_reason(ack["reason"]) if ack.get("reason") else "")
            self.pending = None
        if epoch_changed:
            self.clear_inputs()
            # 尚未確認的舊命令不能跨過控制權世代變更。
            self.pending = None
        if mode != "manual":
            self.clear_inputs()
        if reconnect:
            # 初次配對、電腦重啟或網路重連一律先停止，不恢復先前油門。
            self.clear_inputs()
            self.pending = None
            self.request_action("stop")
        return True

    def request_action(self, name, now=None):
        if name not in ACTION_LABELS:
            return False
        # K 必須由操作者明確停止後再選取；拒絕時不偷換模式或清掉行駛操作。
        if name in ("park", "search") and self.connected and self.mode != "standby":
            self.action_result = "請先按空白鍵停止，確認待命後再按 " + ("K" if name == "park" else "G")
            return False
        if name == "patrol" and self.connected and self.mode in ("park", "search", "offer"):
            self.action_result = "請先按空白鍵停止目前任務，確認待命後再按 P"
            return False
        offer_id = None
        if name in ("park_accept", "park_decline"):
            remaining = self.offer_remaining(now)
            if remaining is None or remaining <= 0:
                self.action_result = "目前沒有可回覆的停車通知，或本次通知已到期／已處理"
                return False
            offer_id = self.mission["offer_id"]
        self.clear_inputs()
        if not self.connected or self.session is None:
            self.pending = None
            self.action_result = "尚未連線；請在連線後重新操作"
            return False
        if name in ("manual", "patrol", "park", "search") and self.capabilities.get(name) is not True:
            self.action_result = ("尚未完成停車設定" if name == "park" and self.capabilities.get(name) is False
                                  else "車端尚未開放" + action_label(name, self.parking))
            return False
        if name != "stop" and self.stop_held:
            self.action_result = "請先放開空白鍵；目前維持停止要求"
            return False
        if self.pending is not None and self.pending["name"] == name:
            return True
        if name in ("stop", "manual", "park_accept", "park_decline"):
            self._dismissed_offer = self._offer_key
        self.last_action_id = max(self.last_action_id + 1, self.next_seq,
                                  self.remote_last_seq + 1, 1)
        self.pending = {"id": self.last_action_id, "name": name, "epoch": self.epoch}
        if offer_id is not None:
            self.pending["offer_id"] = offer_id
        self.action_result = action_label(name, self.parking) + "：等待車端確認"
        return True

    def key_down(self, key):
        key = key.lower()
        if key == "space":
            if not self.stop_held:
                self.request_action("stop")
            self.stop_held = True
            return
        if key not in ("w", "a", "s", "d"):
            return
        if key in self.blocked_keys:
            return
        if (key in ("w", "a", "s", "d") and self.connected
                and self.mode == "manual" and self.manual_ready
                and self.pending is None and not self.stop_held):
            self.held.add(key)
        else:
            self.blocked_keys.add(key)

    def key_up(self, key):
        self.held.discard(key.lower())
        self.blocked_keys.discard(key.lower())
        if key.lower() == "space":
            self.stop_held = False

    def next_packet(self, now):
        self.check_timeout(now)
        if not self.connected or self.session is None:
            return None
        if (self.pending and self.pending["name"] in ("park_accept", "park_decline")
                and (self.mode != "offer" or self.pending.get("offer_id") != self.epoch
                     or not self._mission_offer_valid
                     or self._offer_deadline is None or now >= self._offer_deadline)):
            self.pending = None
            self.request_action("stop")
        if self.mode == "manual" and self.manual_ready and self.pending is None and not self.stop_held:
            if ("w" in self.held) != ("s" in self.held):
                self.esc = max(ESC_MIN, min(ESC_MAX, self.esc + (1 if "w" in self.held else -1)))
            else:
                self.esc = _toward(self.esc, ESC_STOP)
            if ("a" in self.held) != ("d" in self.held):
                self.servo = max(SERVO_MIN, min(SERVO_MAX, self.servo + (-1 if "a" in self.held else 1)))
            else:
                self.servo = _toward(self.servo, SERVO_CENTER)
        else:
            self.clear_inputs()
        packet = {"v": 1, "type": "control", "session": self.session,
                  "seq": self.next_seq, "epoch": self.epoch,
                  "esc": self.esc, "servo": self.servo,
                  "action": dict(self.pending) if self.pending else None}
        self.next_seq += 1
        return packet


class ControlWindow:
    def __init__(self, root, sock, bind, port):
        import tkinter as tk
        from tkinter import ttk

        self.root, self.sock = root, sock
        self.state = ControlClientState()
        self.closing = False
        self.shortcut_held = set()
        self._notified_offers = deque(maxlen=32)
        self.last_render = 0.0
        self.network_error = ""
        root.title("智慧車｜第 10 階段控制台")
        root.geometry("860x950")
        root.minsize(800, 880)
        frame = ttk.Frame(root, padding=18)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text="車輛模式、反應式巡邏與自動找位入庫", font=("Microsoft JhengHei", 17, "bold")).pack(anchor="w")
        ttk.Label(frame, text="完成停車設定後，先按空白鍵停止，再按 K 尋找可見車位並規劃入庫。P 巡邏仍保留。",
                  wraplength=640).pack(anchor="w", pady=(5, 12))
        self.status_text = tk.StringVar(value="等待 Jetson 連線")
        self.output_text = tk.StringVar()
        self.reason_text = tk.StringVar()
        self.action_text = tk.StringVar()
        self.health_text = tk.StringVar()
        self.video_text = tk.StringVar()
        self.patrol_text = tk.StringVar()
        self.parking_text = tk.StringVar()
        self.mission_text = tk.StringVar()
        self.perception_text = tk.StringVar()
        ttk.Label(frame, textvariable=self.status_text, font=("Microsoft JhengHei", 13, "bold")).pack(anchor="w")
        ttk.Label(frame, textvariable=self.output_text).pack(anchor="w", pady=(5, 8))
        row = ttk.Frame(frame)
        row.pack(fill="x", pady=5)
        self.manual_button = ttk.Button(row, text="M　手動接管", takefocus=False,
                                        command=lambda: self._action("manual"))
        self.manual_button.pack(side="left", padx=(0, 8))
        ttk.Button(row, text="空白鍵　停止待命", takefocus=False,
                   command=lambda: self._action("stop")).pack(side="left", padx=(0, 8))
        self.reset_button = ttk.Button(row, text="R　解除故障", takefocus=False,
                                       command=lambda: self._action("reset"))
        self.reset_button.pack(side="left")
        row2 = ttk.Frame(frame)
        row2.pack(fill="x", pady=5)
        self.patrol_button = ttk.Button(row2, text="P　反應式巡邏", takefocus=False,
                                        command=lambda: self._action("patrol"))
        self.patrol_button.pack(side="left", padx=(0, 8))
        self.park_button = ttk.Button(row2, text="K　自動找位入庫", takefocus=False,
                                      command=lambda: self._action("park"))
        self.park_button.pack(side="left")
        self.search_button = ttk.Button(row2, text="G　巡邏找位", takefocus=False,
                                        command=lambda: self._action("search"))
        self.search_button.pack(side="left", padx=8)
        mission_group = ttk.LabelFrame(frame, text="找位與停車決定", padding=8)
        mission_group.pack(fill="x", pady=5)
        ttk.Label(mission_group, textvariable=self.mission_text, wraplength=740).pack(anchor="w")
        mission_buttons = ttk.Frame(mission_group)
        mission_buttons.pack(fill="x", pady=(5, 0))
        self.accept_button = ttk.Button(mission_buttons, text="Y　同意入庫", takefocus=False,
                                        command=lambda: self._action("park_accept"))
        self.decline_button = ttk.Button(mission_buttons, text="N　略過繼續找位", takefocus=False,
                                         command=lambda: self._action("park_decline"))
        self.accept_button.pack(side="left", padx=(0, 8))
        self.decline_button.pack(side="left")
        ttk.Label(frame, text="W／S：前進／倒退　A／D：轉向\n"
                  "M 手動接管；P 巡邏；K 直接入庫；G 巡邏找位後通知。自主行駛時 WASD 不控制車輛。\n"
                  "手動就緒後再按 WASD；切換時先回中立，舊按鍵不延續。\n"
                  "空白鍵立即要求停止；切換到其他視窗也會要求停止待命。",
                  wraplength=640).pack(anchor="w", pady=(10, 8))
        ttk.Label(frame, textvariable=self.action_text, wraplength=640).pack(anchor="w")
        ttk.Label(frame, textvariable=self.reason_text, wraplength=640).pack(anchor="w", pady=4)
        ttk.Label(frame, textvariable=self.health_text, wraplength=640).pack(anchor="w")
        ttk.Label(frame, textvariable=self.video_text, wraplength=660).pack(anchor="w", pady=6)
        ttk.Label(frame, textvariable=self.patrol_text, wraplength=680).pack(anchor="w", pady=4)
        ttk.Label(frame, textvariable=self.parking_text, wraplength=720).pack(anchor="w", pady=4)
        ttk.Label(frame, textvariable=self.perception_text, wraplength=680).pack(anchor="w", pady=4)
        ttk.Label(frame, text="巡邏依四向飛行時間測距（ToF）；自動入庫使用雷達定位與四向測距保護。車位橫向向左為正。",
                  wraplength=680).pack(anchor="w", pady=4)
        ttk.Label(frame, text="等待車端主動連入：{}:{}（UDP）".format(bind, port)).pack(anchor="w", pady=(10, 0))
        root.bind("<KeyPress>", self._key_down)
        root.bind("<KeyRelease>", self._key_up)
        root.bind("<FocusOut>", self._focus_out)
        root.protocol("WM_DELETE_WINDOW", self._close)
        root.after_idle(root.focus_set)
        root.after(0, self._poll)

    def _action(self, name):
        self.state.request_action(name)
        # 滑鼠點擊按鈕後，讓 WASD／空白鍵由主視窗處理。
        self.root.focus_set()
        self._render()

    def _key_down(self, event):
        key = event.keysym.lower()
        if key == "space" or key in SHORTCUT_ACTIONS:
            if key not in self.shortcut_held:
                self.shortcut_held.add(key)
                if key == "space":
                    self.state.key_down("space")
                else:
                    self.state.request_action(SHORTCUT_ACTIONS[key])
            return "break"
        if key in ("w", "a", "s", "d"):
            self.state.key_down(key)
            return "break"

    def _key_up(self, event):
        key = event.keysym.lower()
        self.shortcut_held.discard(key)
        self.state.key_up(key)
        if key in ("space", "w", "a", "s", "d") or key in SHORTCUT_ACTIONS:
            return "break"

    def _focus_out(self, _event):
        self.root.after_idle(self._check_focus)

    def _check_focus(self):
        if not self.closing and self.root.focus_get() is None:
            self.state.request_action("stop")

    def _send(self):
        packet = self.state.next_packet(time.monotonic())
        if packet is None:
            return
        try:
            self.sock.sendto(json.dumps(packet, separators=(",", ":")).encode("utf-8"),
                             self.state.address)
            self.network_error = ""
        except OSError as exc:
            self.network_error = "傳送失敗：" + str(exc)
            self.state.clear_inputs()

    def _poll(self):
        if self.closing:
            return
        for _ in range(64):
            try:
                data, address = self.sock.recvfrom(16384)
            except BlockingIOError:
                break
            except OSError as exc:
                self.network_error = "接收失敗：" + str(exc)
                break
            try:
                message = json.loads(data.decode("utf-8"))
            except (ValueError, UnicodeError, RecursionError):
                continue
            self.state.observe(message, address, time.monotonic())
        self._send()
        now = time.monotonic()
        if now - self.last_render >= 0.1:
            self._render()
            self.last_render = now
        self.root.after(SEND_INTERVAL_MS, self._poll)

    def _render(self):
        st = self.state
        endpoint = "{}:{}".format(*st.address) if st.address else "尚未配對"
        mode_label = MODE_LABELS.get(st.mode, st.mode)
        if st.mode == "manual":
            mode_label = "手動就緒" if st.manual_ready else "手動交接中，請稍候"
        elif st.mode == "patrol":
            mode_label = "反應式巡邏" if st.patrol.get("ready") is True else "巡邏交接／等待中"
        elif st.mode == "park":
            mode_label = ("舊版直線倒車入庫" if st.parking.get("range_source") == "rear_tof" else "自動找位入庫") if st.parking.get("ready") is True else "入庫交接／準備中"
        elif st.mode == "offer":
            mode_label = "停止等待你的停車決定" if st.mission.get("state") == "awaiting" else "停止並確認車位"
        if not st.connected:
            last_mode = action_label("park", st.parking) if st.mode == "park" else MODE_LABELS.get(st.mode, st.mode)
            mode_label = "未知（最後模式：{}）".format(last_mode)
        self.status_text.set("{}｜模式：{}｜車端：{}".format(
            "已連線" if st.connected else "未連線", mode_label, endpoint))
        self.output_text.set("電腦命令：油門 {}／轉向 {}　（巡邏／入庫時僅送中立心跳，由車端決定輸出）".format(st.esc, st.servo))
        self.action_text.set(st.action_result)
        self.reason_text.set("狀態原因：" + (self.network_error or describe_reason(st.reason)))
        distances = st.health.get("tof_mm", {})
        distances = distances if isinstance(distances, dict) else {}
        readings = []
        for key, label in (("front", "前"), ("rear", "後"), ("left", "左"), ("right", "右")):
            value = distances.get(key)
            readings.append(label + "：" + (str(value) + " mm" if value is not None else "未知"))
        self.health_text.set("M4：{}　四向距離：{}".format(
            "已連線" if st.health.get("m4_connected") else "未連線", "　".join(readings)))
        video_ok = st.connected and st.video.get("ready") is True
        frame_age = st.video.get("frame_age_ms")
        age_label = "未知" if not isinstance(frame_age, (int, float)) else "{:.0f} 毫秒".format(frame_age)
        self.video_text.set("影像：{}；影格年齡：{}\n{}".format(
            "接收確認正常" if video_ok else "尚未就緒／失聯", age_label,
            "" if video_ok else describe_reason(st.video.get("reason") or "video_unconfirmed")))
        patrol_label = PATROL_LABELS.get(st.patrol.get("state"), "未知") if st.connected else "未知（連線中斷）"
        self.patrol_text.set("巡邏狀態：{}；原因：{}".format(
            patrol_label, describe_reason(st.patrol.get("reason"))))
        self.parking_text.set(parking_text(st.parking, current=st.connected))
        if st.connected and st.capabilities.get("park") is False:
            self.parking_text.set("自動入庫：尚未完成停車設定；K 暫停使用")
        self.perception_text.set(perception_text(st.perception, current=st.connected))
        self.manual_button.configure(state="normal" if st.connected and st.capabilities.get("manual") is True else "disabled")
        self.patrol_button.configure(state="normal" if st.connected and st.capabilities.get("patrol") is True else "disabled")
        self.park_button.configure(state="normal" if st.connected and st.capabilities.get("park") is True else "disabled",
                                   text="K　舊版直線入庫" if st.connected and st.parking.get("range_source") == "rear_tof" else "K　自動找位入庫")
        self.reset_button.configure(state="normal" if st.connected and st.mode == "fault" else "disabled")
        self._render_mission()

    def _render_mission(self):
        st, now = self.state, time.monotonic()
        # 亦相容測試／舊的嵌入式視窗，只在具有新增控制元件時更新。
        if hasattr(self, "mission_text"):
            self.mission_text.set(st.mission_status(now))
        remaining = st.offer_remaining(now)
        can_reply = remaining is not None and remaining > 0 and not self.closing
        for name in ("accept_button", "decline_button"):
            if hasattr(self, name):
                getattr(self, name).configure(state="normal" if can_reply else "disabled")
        if hasattr(self, "search_button"):
            self.search_button.configure(state="normal" if st.connected and st.capabilities.get("search") is True
                                         and st.mode == "standby" and not self.closing else "disabled")
        # 提示音只播放一次，不彈出對話框，也不搶走影像視窗焦點。
        if can_reply and hasattr(self, "_notified_offers") and st._offer_key not in self._notified_offers:
            self._notified_offers.append(st._offer_key)
            try:
                self.root.bell()
            except Exception:
                pass

    def _close(self):
        if self.closing:
            return
        self.closing = True
        self.shortcut_held.clear()
        self.state.request_action("stop")
        self._close_send(0)

    def _close_send(self, attempt):
        self._send()
        if attempt < 2:
            self.root.after(40, lambda: self._close_send(attempt + 1))
        else:
            self.sock.close()
            self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description="智慧車電腦端手動、巡邏與自動找位入庫控制；取代 sever(1).py")
    parser.add_argument("--bind", default="0.0.0.0", help="監聽位址，預設所有本機介面")
    parser.add_argument("--port", type=int, default=5000, help="控制 UDP 連接埠，預設 5000")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("連接埠必須介於 1～65535")
    try:
        import tkinter as tk
    except ImportError:
        raise SystemExit("找不到 Tk 視窗套件。Windows 官方 Python 安裝程式請啟用 Tcl/Tk 選項。")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((args.bind, args.port))
        sock.setblocking(False)
    except OSError as exc:
        sock.close()
        raise SystemExit("無法監聽 UDP {}：{}。請先關閉舊的 sever(1).py。".format(args.port, exc))
    print("智慧車控制台等待 Jetson 主動連入 {}:{}；M 手動、P 巡邏、K 入庫、G 巡邏找位、Y 同意、N 略過、空白鍵停止。".format(args.bind, args.port))
    try:
        root = tk.Tk()
        window = ControlWindow(root, sock, args.bind, args.port)
        root.mainloop()
    except KeyboardInterrupt:
        if 'window' in locals():
            window.state.request_action("stop")
            window._send()
    finally:
        sock.close()


if __name__ == "__main__":
    main()
