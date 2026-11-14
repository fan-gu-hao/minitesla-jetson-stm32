# -*- coding: utf-8 -*-
"""本車固定參數（2026-10-12）：依本次提供的安裝與轉彎資料寫入。

執行與開機時不需要輸入；不讀取舊 parking_config.json。
長度以公尺保存，角度使用度。左右轉彎半徑 48 公分 = 0.480 公尺。
雷達原始 0° 朝車頭、原始角度順時針、雷達位於車身中心線。
車尾至後軸中心 14 公分。車長、車寬、軸距、雷達縱向位置沿用既有紀錄。

使用者允許放大展示車位；本版採內寬 180、內深 130 公分。
車位是本版選定的展示配置，不是已完成搭建的量測紀錄。
啟用開放 G 巡邏找位與 K 停車指令；開機仍待命，不會自動行駛。

右側正向：lidar_y_from_center_m > 0 表示車身中心線右方。
轉彎半徑是對應命令下後軸中心的軌跡半徑；改動轉向 PWM 時需同步校正。
修改此檔案後必須重新啟動服務，才會更新執行中的參數。
"""

# 這是唯一固定參數來源；不要在其他模組重複填相同資料。
VEHICLE_SETTINGS = {
    # 本次資料已填入；暫時停用自動停車可改為 False。
    "enabled": True,

    # 雷達安裝：採用本次確認的朝向、順時針及置中位置。
    "lidar_yaw_deg": 0.0,  # 原始 0° 對應的車體方向。
    "lidar_clockwise": True,  # 原始角度遞增方向：順時針 True、逆時針 False。
    "lidar_x_from_tail_m": 0.300,  # 從車尾向前量至雷達旋轉中心。
    "lidar_y_from_center_m": 0.0,  # 由車身中心線量，右正左負；置中填 0.0。

    # 車身與轉向幾何：後懸長不是軸距。
    "rear_overhang_m": 0.140,  # 車尾至後軸中心。
    "left_turn_radius_m": 0.480,  # steering_left_pwm 命令下的後軸中心半徑。
    "right_turn_radius_m": 0.480,  # steering_right_pwm 命令下的後軸中心半徑。

    # 沿用的舵機與 ESC 命令；PWM 數值不是轉向角度。
    "steering_left_pwm": 128,
    "steering_center_pwm": 152,
    "steering_right_pwm": 176,
    "forward_esc": 154,
    "reverse_esc": 150,

    # 沿用專案既有尺寸；若現場量的是整車外框，請在此更新。
    "body_length_m": 0.474,
    "body_width_m": 0.328,
    "wheelbase_m": 0.275,

    # 本版展示車位：請依內寬 180、內深 130 公分搭設三面實體擋板。
    "bay_width_m": 1.8,
    "bay_depth_m": 1.3,

    # 巡邏找位任務：通知後等待決定，未回覆則停止待命。
    "mission_confirm_s": 15.0,
    "mission_search_timeout_s": 120.0,
    "mission_offer_acquire_s": 5.0,
    "mission_ignore_s": 10.0,
    "mission_clear_s": 1.0,
    "mission_report_timeout_s": 0.5,

    # 下方沿用第九階段的規劃與保護參數，一般不需為量尺寸而修改。
    "bay_dimension_tolerance_m": 0.15,
    "target_back_clearance_m": 0.6,
    "margin_m": 0.18,
    "tof_clearance_mm": 450,
    "max_speed_m_s": 0.30,  # 定位估計運動超限時中止；不是定速控制。
    "max_yaw_rate_rad_s": 0.8,
    "max_scan_age_s": 0.25,
    "proposal_timeout_s": 0.35,
    "min_scan_points": 80,
    "min_new_points": 60,
    "scan_range_m": 6.0,
    "search_timeout_s": 20.0,
    "mission_timeout_s": 90.0,
    "planning_timeout_s": 5.0,
    "prepare_s": 0.5,
    "gear_wait_s": 0.6,
    "gear_timeout_s": 3.0,
    "settle_s": 0.8,
    "settle_timeout_s": 4.0,
    "min_stable_samples": 4,
    "position_tolerance_m": 0.08,
    "yaw_tolerance_deg": 6.0,
    "max_cross_track_m": 0.18,
    "no_progress_s": 4.0,
    "min_progress_m": 0.03,
    "lookahead_m": 0.22,
    "grid_resolution_m": 0.08,
    "max_nodes": 20000,
    "time_budget_s": 3.0,
}
