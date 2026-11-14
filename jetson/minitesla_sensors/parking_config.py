"""自動找位入庫的設定；不匯入數值函式庫，也不操作硬體。"""
import copy
import math
from . import vehicle_config


def defaults():
    """每次回傳獨立副本；現場量測值統一寫在 vehicle_config.py。"""
    return copy.deepcopy(vehicle_config.VEHICLE_SETTINGS)


def _finite(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def validate_settings(settings, require_enabled=False):
    """回傳獨立的完整設定；啟用前必須填妥實際安裝方向與轉向幾何。"""
    if not isinstance(settings, dict):
        raise ValueError("停車設定必須是字典")
    cfg = defaults()
    cfg.update(copy.deepcopy(settings))
    if not isinstance(cfg["enabled"], bool) or not isinstance(cfg["lidar_clockwise"], bool):
        raise ValueError("enabled 與 lidar_clockwise 必須為布林值")
    if require_enabled and not cfg["enabled"]:
        raise ValueError("自動停車尚未啟用")
    required = ("lidar_yaw_deg", "lidar_y_from_center_m", "rear_overhang_m", "left_turn_radius_m", "right_turn_radius_m")
    missing = [k for k in required if cfg[k] is None]
    if cfg["enabled"] and missing:
        raise ValueError("啟用前請完成停車設定：" + "、".join(missing))
    ranges = {
        "lidar_yaw_deg": (-360, 360), "lidar_x_from_tail_m": (-0.1, 0.65),
        "lidar_y_from_center_m": (-0.3, 0.3), "rear_overhang_m": (0.01, 0.18),
        "left_turn_radius_m": (0.25, 3), "right_turn_radius_m": (0.25, 3),
        "body_length_m": (0.35, 0.65), "body_width_m": (0.25, 0.5),
        "wheelbase_m": (0.2, 0.4), "bay_width_m": (1.3, 3),
        "bay_depth_m": (1.3, 3), "bay_dimension_tolerance_m": (0.03, 0.2),
        "target_back_clearance_m": (0.5, 0.8), "margin_m": (0.12, 0.3),
        "tof_clearance_mm": (450, 600), "max_speed_m_s": (0.05, 0.30),
        "max_yaw_rate_rad_s": (0.1, 0.8), "max_scan_age_s": (0.1, 0.25),
        "proposal_timeout_s": (0.2, 0.35), "scan_range_m": (2, 6),
        "search_timeout_s": (2, 30), "mission_timeout_s": (10, 120),
        "planning_timeout_s": (3.5, 5), "prepare_s": (0.4, 2),
        "gear_wait_s": (0.5, 2), "gear_timeout_s": (2, 5),
        "settle_s": (0.5, 2), "settle_timeout_s": (2, 6),
        "position_tolerance_m": (0.04, 0.1), "yaw_tolerance_deg": (2, 8),
        "max_cross_track_m": (0.08, 0.2), "no_progress_s": (1, 5),
        "min_progress_m": (0.02, 0.1), "lookahead_m": (0.12, 0.4),
        "grid_resolution_m": (0.04, 0.1), "time_budget_s": (0.2, 3),
        "mission_confirm_s": (5.0, 60.0), "mission_search_timeout_s": (30.0, 300.0),
        "mission_offer_acquire_s": (2.0, 10.0), "mission_ignore_s": (5.0, 60.0),
        "mission_clear_s": (0.5, 5.0), "mission_report_timeout_s": (0.35, 0.5),
    }
    for key, (lo, hi) in ranges.items():
        value = cfg[key]
        if value is None and key in required and not cfg["enabled"]:
            continue
        if not _finite(value) or not lo <= value <= hi:
            raise ValueError("%s 必須介於 %s 與 %s" % (key, lo, hi))
    for key, lo, hi in (("steering_left_pwm", 94, 210), ("steering_right_pwm", 94, 210),
                        ("forward_esc", 153, 155), ("reverse_esc", 148, 151),
                        ("min_scan_points", 60, 360), ("min_new_points", 40, 300),
                        ("min_stable_samples", 3, 12), ("max_nodes", 1000, 20000)):
        v = cfg[key]
        if not isinstance(v, int) or isinstance(v, bool) or not lo <= v <= hi:
            raise ValueError("%s 必須為 %d～%d 的整數" % (key, lo, hi))
    if cfg["steering_center_pwm"] != 152 or isinstance(cfg["steering_center_pwm"], bool):
        raise ValueError("沿用現有韌體，舵機中位必須為 152")
    if (cfg["steering_left_pwm"] - 152) * (cfg["steering_right_pwm"] - 152) >= 0:
        raise ValueError("左右轉向命令必須分別位於中位 152 的兩側")
    if cfg["rear_overhang_m"] is not None:
        if cfg["rear_overhang_m"] + cfg["wheelbase_m"] >= cfg["body_length_m"]:
            raise ValueError("後懸長加軸距必須小於車長")
    if cfg["target_back_clearance_m"] + cfg["body_length_m"] + 0.15 >= cfg["bay_depth_m"]:
        raise ValueError("車位深度不足以容納後方淨空、車長與入口餘量")
    if cfg["bay_width_m"] <= cfg["body_width_m"] + 2 * cfg["tof_clearance_mm"] / 1000:
        raise ValueError("車位寬度不足以保留四向測距停止門檻")
    if cfg["gear_wait_s"] >= cfg["gear_timeout_s"] or cfg["settle_s"] >= cfg["settle_timeout_s"]:
        raise ValueError("穩定等待時間必須小於其逾時時間")
    if cfg["mission_offer_acquire_s"] <= cfg["prepare_s"]:
        raise ValueError("找位停穩確認期限必須大於中立準備時間")
    if cfg["mission_search_timeout_s"] <= cfg["mission_confirm_s"] + cfg["mission_offer_acquire_s"]:
        raise ValueError("找位總期限必須大於停穩與等待決定的時間")
    if cfg["min_new_points"] > cfg["min_scan_points"]:
        raise ValueError("min_new_points 不可大於 min_scan_points")
    if cfg["left_turn_radius_m"] is not None and cfg["right_turn_radius_m"] is not None:
        cfg["min_turn_radius_m"] = max(cfg["left_turn_radius_m"], cfg["right_turn_radius_m"])
    if cfg["rear_overhang_m"] is not None and cfg["lidar_y_from_center_m"] is not None:
        cfg["lidar_origin_body_m"] = [cfg["lidar_x_from_tail_m"] - cfg["rear_overhang_m"],
                                      -cfg["lidar_y_from_center_m"]]
    cfg["configuration_complete"] = not missing
    return cfg


def load_settings():
    """直接讀取程式內的固定參數；不讀 JSON、不詢問、不寫檔。"""
    try:
        return validate_settings(defaults())
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("請修正 minitesla_sensors/vehicle_config.py：" + str(exc)) from exc
