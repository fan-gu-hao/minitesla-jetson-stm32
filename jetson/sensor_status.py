#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""讀取主程式狀態快照，絕不另開 UART。"""
import argparse
import json
import math
from pathlib import Path
import time

MODE_LABELS = {"latched": "保護鎖定", "armed_neutral": "中立待命", "running": "輸出行駛命令", "unknown": "未知"}
CONTROL_LABELS = {"standby": "待命", "manual": "手動", "patrol": "反應式巡邏", "park": "自動找位入庫", "search": "巡邏找位", "offer": "等待停車決定", "fault": "故障待命"}
PATROL_LABELS = {"idle": "未啟動", "handoff": "中立交接", "cruise": "向前巡行",
                 "turn_left": "向左繞行", "turn_right": "向右繞行", "blocked": "停止等待空間",
                 "stopped": "已停止，清除障礙後需重新按 P"}
PARKING_LABELS = {"idle": "未啟動", "handoff": "中立交接", "preparing": "中立準備中",
                  "searching": "掃描目前可見範圍，尋找車位", "acquiring": "確認車位位置穩定",
                  "planning": "停止並規劃入庫路徑", "aligning": "向前調整入庫位置",
                  "reversing": "倒車入庫中", "gear_wait": "中立等待換向",
                  "settling": "停止並確認停妥",
                  "complete": "入庫完成", "aborted": "已停止，需重新操作"}
MISSION_LABELS = {"idle": "未啟動", "searching": "巡邏找位", "verifying": "停止確認車位",
                  "awaiting": "等待停車決定", "skipping": "略過後繼續找位", "parking": "入庫中",
                  "completed": "任務完成", "aborted": "任務已停止"}
CONTROL_REASON_LABELS = {
    "park_approved_target_missing": "已同意的車位資料缺失，停止入庫",
    'mission_found': '發現候選車位，先停止確認',
    'mission_ready': '車位確認完成，等待你的停車決定',
    'mission_skipping': '已略過本車位，繼續巡邏找位',
    'mission_invalid': '找位任務資料無效，停止待命',
    'mission_target_missing': '缺少本次車位資料，停止待命',
    'mission_acquire_timeout': '車位穩定確認逾時，停止待命',
    'mission_sensor_invalid': '找位感測資料無效，停止待命',
    'mission_pose_lost': '無法持續定位同一車位，停止找位任務',
    'mission_speed_exceeded': '估計車輛移動過快，停止找位任務',

    'search_selected': '已選擇巡邏找位',
    'search_input_timeout': '巡邏找位監督心跳逾時',
    'offer_input_timeout': '停車決定監督心跳逾時',
    'mission_searching': '巡邏中，尋找雷達可辨識的車位',
    'mission_candidate_found': '發現候選車位，先停止確認',
    'mission_verifying': '保持中立，確認同一車位穩定',
    'mission_awaiting_decision': '車輛已停止，等待你的停車決定',
    'mission_declined': '已略過本次車位，繼續找位',
    'mission_accepted': '已同意停入本次車位',
    'mission_search_timeout': '找位任務時間已到，停止待命',
    'mission_decision_timeout': '停車決定時間已到，停止待命',
    'mission_verify_timeout': '車位穩定確認逾時，停止待命',
    'mission_worker_timeout': '找位程序回報逾時，停止待命',
    'mission_source_stale': '找位雷達來源資料逾時，停止待命',
    'mission_target_lost': '無法持續辨認本次車位，停止待命',
    'mission_report_invalid': '找位程序資料不合規，停止待命',
    'mission_not_stationary': '車輛尚未穩定停止，不能確認停車',
    'mission_no_offer': '目前沒有有效的停車通知',
    'mission_offer_mismatch': '停車通知已變更，忽略舊決定',
    'mission_requires_standby': '請先按空白鍵停止，確認待命後再按 G',
    'mission_m4_reboot': 'M4 已重新啟動，停止找位任務',
    'mission_path_blocked': '找位行駛空間不足，停止待命',
    'mission_lidar_unavailable': '雷達資料無效或逾時，停止找位任務',
    'mission_target_invalid': '本次車位資料無效，停止待命',
    'mission_vehicle_moved': '等待決定期間車輛位置改變，停止待命',
    'mission_internal_error': '找位程序內部狀態錯誤，停止待命',
    'mission_pose_jump': '車位定位突然改變，停止待命',
    'mission_ambiguous_bay': '無法唯一確認車位，停止待命',
    'mission_tof_invalid_front': '前方測距無效或逾時，停止找位任務',
    'mission_tof_invalid_rear': '後方測距無效或逾時，停止找位任務',
    'mission_tof_invalid_left': '左方測距無效或逾時，停止找位任務',
    'mission_tof_invalid_right': '右方測距無效或逾時，停止找位任務',

    'park_selected': '已選擇入庫模式',
    'park_input_timeout': '入庫監督心跳逾時',
    'park_camera_unavailable': '入庫影像辨識資料無效或逾時',
    'park_requires_standby': '請先按空白鍵停止，確認待命後再按 K',
    'park_ready': '入庫已就緒',
    'park_preparing': '保持中立並確認測距',
    'park_reversing': '倒車入庫中',
    'park_settling': '停止並確認停妥',
    'park_complete': '已完成入庫',
    'park_sensor_invalid': '入庫測距資料無效或逾時',
    'park_clock_invalid': '入庫時鐘資料異常',
    'park_m4_reboot': 'M4 已重新啟動，停止入庫',
    'park_sample_regression': '入庫測距樣本時序倒退',
    'park_sample_invalid': '尚未取得新的入庫測距樣本',
    'park_start_out_of_range': '後方起始距離不在允許範圍',
    'park_path_blocked': '入庫路徑空間不足',
    'park_prepare_timeout': '中立準備逾時',
    'park_no_progress': '入庫路段沒有進展，已停止',
    'park_wrong_direction': '後方距離持續增加，方向異常，已停止',
    'park_distance_jump': '後方距離異常跳變',
    'park_timeout': '入庫總時間逾時',
    'park_settle_timeout': '停車位置未能穩定',
    'park_target_out_of_range': '目前位置超出入庫完成範圍',
    'video_unconfirmed': '尚未收到電腦端影格確認，請先啟動新版影像接收器',
    'video_disconnected': '影像連線中斷；恢復後按 R，再選 M、P、G 或 K',
    'video_ack_timeout': '影格確認逾時；恢復後按 R，再選 M、P、G 或 K',
    'video_frame_stale': '影格已過期，請檢查攝影機、推論及網路',
    'video_sender_stopped': '影像傳送程序已停止，請重新啟動車端主程式',
    'startup': '啟動待命',
    'manual_selected': '已選擇手動模式',
    'operator_stop': '操作者停止',
    'stopped': '已停止',
    'fault_reset': '已清除故障，等待選擇模式',
    'shutdown': '程式結束',
    'manual_input_timeout': '手動命令逾時',
    'm4_offline': 'M4 未連線或遙測逾時',
    'm4_guard_latched': 'M4 保護鎖定',
    'm4_state_unknown': 'M4 狀態未知',
    'm4_transport_error': 'M4 串列通訊錯誤',
    'reset_required': '請先清除故障',
    'neutral_required': '請先釋放行駛按鍵',
    'stop_before_reset': '請先停止再清除故障',
    'feature_unavailable': '此自主功能尚未實作',
    'epoch_mismatch': '模式已變更，請重新操作',
    'action_id_conflict': '模式請求編號衝突',
    'stale_action': '模式請求已過期',
    'unknown_action': '未知模式請求',
    'tof_invalid_front': '前方測距無效',
    'tof_invalid_rear': '後方測距無效',
    'tof_invalid_left': '左方測距無效',
    'tof_invalid_right': '右方測距無效',
    'patrol_selected': '已選擇反應式巡邏',
    'patrol_input_timeout': '巡邏監督心跳逾時',
    'patrol_camera_unavailable': '巡邏影像辨識資料無效或逾時',
    'patrol_path_blocked': '巡邏路徑受阻',
    'patrol_turn_timeout': '繞行逾時',
    'patrol_sensor_invalid': '巡邏測距資料無效或逾時',
    'patrol_wait_clear': '等待通行空間',
    'patrol_ready': '巡邏已就緒',
    'patrol_cruise': '前方空間充足，向前巡行',
    'patrol_turn_left': '向左繞行',
    'patrol_turn_right': '向右繞行',
    'patrol_clock_invalid': '巡邏時鐘資料異常',
    'auto_park_not_configured': '尚未完成停車設定',
    'park_lidar_unavailable': '雷達資料無效或逾時，停止入庫',
    'park_pose_lost': '無法持續辨認車位位置，停止入庫',
    'park_pose_jump': '車位定位突然改變，停止入庫',
    'park_ambiguous_bay': '可見車位無法唯一確認，請調整車輛位置',
    'park_no_bay': '目前可見範圍未找到合適車位',
    'park_no_path': '找不到具有足夠空間的入庫路徑',
    'park_worker_stale': '入庫規劃資料逾時，停止入庫',
    'park_gear_timeout': '中立換向確認逾時，停止入庫',
    'park_speed_exceeded': '估計行駛速度超出停車設定，停止入庫',
    'park_calibration_invalid': '停車校正資料無效，請檢查設定',
    'park_searching': '停止掃描目前可見範圍，尋找車位',
    'park_acquiring': '確認車位位置與方向穩定',
    'park_planning': '保持中立，規劃入庫路徑',
    'park_aligning': '沿規劃路徑向前調整位置',
    'park_gear_wait': '等待中立輸出確認後換向',
    'park_tracking_error': '車輛偏離規劃路徑，停止入庫',
    'park_internal_error': '停車程序內部狀態錯誤，停止入庫',
    'auto_park_m4_reboot': 'M4 已重新啟動，停止自動入庫',
    'auto_park_path_blocked': '車輛周圍空間不足，停止自動入庫',
    'auto_park_plan_timeout': '入庫路徑規劃逾時，已停止',
    'auto_park_report_invalid': '停車程序回報資料不合規，已停止',
    'auto_park_search_timeout': '尋找可見車位逾時，已停止',
    'auto_park_source_stale': '入庫定位來源資料逾時，已停止',
    'auto_park_state_invalid': '停車程序狀態不一致，已停止',
    'auto_park_timeout': '自動入庫總時間逾時，已停止',
    'auto_park_worker_timeout': '停車規劃程序回報逾時，已停止',
    'auto_park_tof_invalid_front': '前方測距資料無效或逾時，停止自動入庫',
    'auto_park_tof_invalid_rear': '後方測距資料無效或逾時，停止自動入庫',
    'auto_park_tof_invalid_left': '左方測距資料無效或逾時，停止自動入庫',
    'auto_park_tof_invalid_right': '右方測距資料無效或逾時，停止自動入庫',
}
REASON_LABELS = {
    "uart_unseen": "尚未收到控制命令", "uart_stale": "控制命令逾時", "uart_range": "控制命令超出範圍",
    "tof_unseen": "測距尚未回報", "tof_status": "測距狀態異常", "tof_stale": "測距逾時",
    "obstacle": "障礙物過近", "wait_clear": "等待障礙清除", "wait_neutral": "等待中立命令",
    "wait_new_throttle": "等待新油門命令", "telemetry_unavailable": "遙測未就緒", "unseen": "尚未收到",
    "initialization_failed": "初始化失敗", "range_invalid": "量測無效", "stale": "資料逾時", "ok": "正常",
}


def age_text(value):
    return "未收到" if value is None else "{:.0f} ms".format(value)


def perception_text(perception, current=True):
    count = perception.get("person_count")
    valid_count = isinstance(count, int) and not isinstance(count, bool) and count >= 0
    if not current or perception.get("valid") is not True or not valid_count:
        status = "未知（影像辨識未就緒或資料過期）"
    elif perception.get("person_present") is True and count > 0:
        status = "警示：辨識到 {} 人；依一般障礙物處理".format(count)
    elif perception.get("person_present") is False and count == 0:
        status = "本影格未辨識到人物"
    else:
        status = "未知（人物資料不一致）"
    events = perception.get("event_count")
    if not isinstance(events, int) or isinstance(events, bool) or events < 0:
        events = "未知"
    return "人物：{}；累計警示 {} 次；影格年齡={}".format(status, events, age_text(perception.get("age_ms")))


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
        CONTROL_REASON_LABELS.get(parking.get("reason"), parking.get("reason", "未知")))


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
        label, CONTROL_REASON_LABELS.get(reason, reason or "正常"), position_label, coordinates(pose),
        coordinates(parking.get("target")), number(parking.get("path_remaining_m"), nonnegative=True),
        number(parking.get("progress_m"), nonnegative=True))


def mission_text(mission, current=True, elapsed=0.0):
    if not isinstance(mission, dict):
        mission = {}
    if not current:
        return "找位任務：未知（資料過期）；歷史通知不能用來確認入庫"
    state = mission.get("state", "idle")
    text = "找位任務：{}；原因：{}".format(
        MISSION_LABELS.get(state, "未知"),
        CONTROL_REASON_LABELS.get(mission.get("reason"), mission.get("reason", "未知")))
    if state == "awaiting":
        remaining = mission.get("remaining_s")
        if (isinstance(remaining, (int, float)) and not isinstance(remaining, bool)
                and math.isfinite(remaining)):
            remaining = max(0.0, remaining - max(0.0, elapsed))
            text += "；通知 #{}；剩餘 {:.1f} 秒".format(mission.get("offer_id"), remaining)
            text += "；請在操作台按 Y 入庫／N 略過，逾時停止待命"
        else:
            text += "；倒數未知，不能視為有效通知"
    return text


def render(snapshot):
    m4, tof, lidar, camera = (snapshot.get(name, {}) for name in ("m4", "tof", "lidar", "camera"))
    produced_at = snapshot.get("monotonic_s")
    mono_age = time.monotonic() - produced_at if produced_at is not None else None
    wall_age = time.time() - snapshot["timestamp_unix"] if snapshot.get("timestamp_unix") is not None else None
    # 重開機的單調時鐘會歸零；不能把負年齡截成零而顯示為最新資料。
    file_age = max(mono_age, wall_age, 0.0) if (mono_age is not None and wall_age is not None
                                               and mono_age >= -0.1 and wall_age >= -1.0) else None
    lines = ["MiniTesla 感測狀態（僅讀取檔案）",
             "快照：{}".format("未提供時間" if file_age is None else "{:.2f} 秒前".format(file_age))]
    if file_age is None or file_age > 1.0:
        lines.append("注意：快照未持續更新，以下為歷史資料，不能視為目前狀態。")
    control = snapshot.get("control", {})
    if control:
        control_label = CONTROL_LABELS.get(control.get("mode"), "未知")
        if control.get("mode") == "park" and control.get("parking", {}).get("range_source") == "rear_tof":
            control_label = "舊版直線入庫"
        lines.append("控制權：{}；交接代次={}；手動已就緒={}".format(
            control_label,
            control.get("epoch"), control.get("manual_ready", False)))
        lines.append("控制原因：{}；命令年齡={}；輸出油門／轉向={}/{}".format(
            CONTROL_REASON_LABELS.get(control.get("reason"), control.get("reason", "未知")),
            age_text(control.get("input_age_ms")),
            control.get("output_esc"), control.get("output_servo")))
        video = control.get("video", {})
        lines.append("影像回傳：就緒={}；影格年齡={}；確認年齡={}；斷線次數={}".format(
            video.get("ready", False), age_text(video.get("frame_age_ms")),
            age_text(video.get("ack_age_ms")), video.get("disconnect_count", 0)))
        if video.get("reason"):
            lines.append("影像原因：" + CONTROL_REASON_LABELS.get(video["reason"], video["reason"]))
        current = file_age is not None and file_age <= 1.0
        patrol = control.get("patrol", {})
        lines.append("巡邏：{}；就緒={}；原因={}；監督心跳={}".format(
            PATROL_LABELS.get(patrol.get("state"), "未知") if current else "未知（快照過期）",
            current and patrol.get("ready", False),
            CONTROL_REASON_LABELS.get(patrol.get("reason"), patrol.get("reason", "未知")),
            age_text(patrol.get("heartbeat_age_ms"))))
        lines.append(parking_text(control.get("parking", {}), current=current))
        lines.append(mission_text(control.get("mission", {}), current=current, elapsed=file_age or 0.0))
        if current and control.get("capabilities", {}).get("park") is False:
            lines.append("自動入庫：尚未完成停車設定；K 暫停使用")
        lines.append(perception_text(control.get("perception", {}), current=current))
        lines.append("巡邏依四向飛行時間測距（ToF）；自動入庫使用雷達定位與四向測距保護。車位橫向向左為正。")
        action = control.get("last_action")
        if action:
            lines.append("最近模式命令：{}；已接受={}；原因={}".format(
                {"manual": "手動", "stop": "停止", "reset": "清除故障", "patrol": "巡邏",
                 "search": "巡邏找位", "park_accept": "同意入庫", "park_decline": "略過車位",
                 "park": "舊版直線入庫" if control.get("parking", {}).get("range_source") == "rear_tof" else "自動找位入庫"}.get(action.get("name"), action.get("name")), action.get("accepted"),
                CONTROL_REASON_LABELS.get(action.get("reason"), action.get("reason"))))
    lines.append("M4：{}；模式：{}；距回報：{}".format(
        "已連接" if m4.get("connected") else "未連接／逾時",
        MODE_LABELS.get(m4.get("mode_name"), "未知"), age_text(m4.get("age_ms"))))
    lines.append("保護原因：{}".format("、".join(REASON_LABELS.get(x, str(x)) for x in m4.get("reasons", [])) or "無／尚未回報"))
    for direction, chinese in (("front", "前"), ("rear", "後"), ("left", "左"), ("right", "右")):
        sensor = tof.get(direction, {})
        distance = sensor.get("distance_mm")
        lines.append("{}方：{}；有效={}；年齡={}；原因={}".format(
            chinese, "未知" if distance is None else "{} mm".format(distance),
            sensor.get("valid", False), age_text(sensor.get("age_ms")), REASON_LABELS.get(sensor.get("reason"), "未知")))
    lines.append("雷達：有效={}；點數={}；年齡={}；車體方向已設定={}".format(
        lidar.get("valid", False), len(lidar.get("points", [])), age_text(lidar.get("age_ms")),
        lidar.get("orientation_configured", False)))
    lines.append("相機：有效={}；物件數={}；年齡={}".format(
        camera.get("valid", False), len(camera.get("detections", [])), age_text(camera.get("age_ms"))))
    if snapshot.get("errors"):
        lines.append("來源訊息：{}".format(json.dumps(snapshot["errors"], ensure_ascii=False)))
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="感測狀態監看；不存取 UART")
    parser.add_argument("--path", default=str(Path(__file__).resolve().parent / "sensor_snapshot.json"))
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--json", action="store_true", help="顯示完整快照")
    args = parser.parse_args()
    try:
        while True:
            try:
                with open(args.path, encoding="utf-8") as stream:
                    snapshot = json.load(stream)
                text = json.dumps(snapshot, ensure_ascii=False, indent=2) if args.json else render(snapshot)
            except (OSError, ValueError) as exc:
                text = "尚未取得感測快照：{}；請先啟動 minitesla 主程式。".format(exc)
            if not args.once:
                print("\033[2J\033[H", end="")
            print(text, flush=True)
            if args.once:
                break
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
