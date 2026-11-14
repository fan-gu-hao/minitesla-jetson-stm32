"""已人工對正車位的直線倒車；只產生命令，不直接操作串口。

使用後方固定牆面的距離決定停止位置，不搜尋車位、不修正車身角度，
也不以電子變速器命令推算車速。呼叫者必須在控制鎖內先檢查感測資料
時效、M4 狀態及影像連線。完成或中止後保持中立，直到明確 reset()。
"""

import math
import time
from collections.abc import Mapping


NEUTRAL = 152
DIRECTIONS = ("front", "rear", "left", "right")
MAX_SAMPLE_GAP_S = 0.2


def _number(value):
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return False
    try:
        return math.isfinite(value)
    except (OverflowError, ValueError):
        return False


def _integer(value, minimum=0, maximum=None):
    return (isinstance(value, int) and not isinstance(value, bool)
            and value >= minimum and (maximum is None or value <= maximum))


class StraightReverseParking:
    """讀取完整設定檔的 parking 區段；不另建執行緒。"""

    DEFAULTS = {
        "reverse_esc": 150,
        "target_rear_mm": 600,
        "arrival_min_mm": 500,
        "arrival_max_mm": 650,
        "start_min_mm": 750,
        "start_max_mm": 1800,
        "clearance_mm": 450,
        "prepare_s": 0.5,
        "prepare_timeout_s": 3.0,
        "settle_s": 0.5,
        "settle_timeout_s": 3.0,
        "stable_span_mm": 30,
        "min_samples": 3,
        "max_reverse_s": 8.0,
        "no_progress_s": 2.0,
        "min_progress_mm": 20,
        "wrong_way_mm": 100,
        "max_step_mm": 250,
    }

    def __init__(self, config=None, clock=time.monotonic):
        if config is None:
            config = {}
        if not isinstance(config, Mapping):
            raise ValueError("停車設定必須是物件")
        section = config.get("parking", {})
        if not isinstance(section, Mapping):
            raise ValueError("parking 必須是設定物件")
        self.config = {key: section.get(key, default)
                       for key, default in self.DEFAULTS.items()}
        self._validate_config()
        if not callable(clock):
            raise ValueError("停車時鐘必須可呼叫")
        self._clock = clock
        self.reset()

    def _validate_config(self):
        cfg = self.config
        for name, value in cfg.items():
            if not _number(value):
                raise ValueError("parking.%s 必須是有限數值" % name)
        if not _integer(cfg["reverse_esc"], 148, 151):
            raise ValueError("parking.reverse_esc 必須是 148 至 151 的整數")
        if not _integer(cfg["min_samples"], 3):
            raise ValueError("parking.min_samples 必須是至少 3 的整數")
        if not (400 <= cfg["clearance_mm"] < cfg["arrival_min_mm"]
                <= cfg["target_rear_mm"] <= cfg["arrival_max_mm"]
                < cfg["start_min_mm"] <= cfg["start_max_mm"]):
            raise ValueError("距離必須符合：400 ≤ 淨空 < 到位下限 ≤ 目標 ≤ 到位上限 < 起點下限 ≤ 起點上限")
        if not 0 < cfg["stable_span_mm"] <= cfg["arrival_max_mm"] - cfg["arrival_min_mm"]:
            raise ValueError("穩定距離跨度必須大於零且不超過到位區間")
        if not 0 < cfg["prepare_s"] < cfg["prepare_timeout_s"]:
            raise ValueError("準備時間必須符合：0 < 穩定時間 < 逾時時間")
        if not 0 < cfg["settle_s"] < cfg["settle_timeout_s"]:
            raise ValueError("到位時間必須符合：0 < 穩定時間 < 逾時時間")
        if not 0 < cfg["no_progress_s"] <= cfg["max_reverse_s"]:
            raise ValueError("倒車時間必須符合：0 < 無進展時間 ≤ 最長倒車時間")
        if not 0 < cfg["min_progress_mm"] <= cfg["wrong_way_mm"] < cfg["max_step_mm"]:
            raise ValueError("距離變化門檻必須符合：0 < 最小進展 ≤ 反向門檻 < 跳變門檻")

    def reset(self, now=None):
        """清除停車鎖定；第一個有效樣本才開始準備計時。"""
        stamp = self._clock() if now is None else now
        if not _number(stamp) or stamp < 0:
            raise ValueError("停車時鐘必須是非負有限數值")
        self._last_at = float(stamp)
        self._state = "idle"
        self._stopped = False
        self._state_at = None
        self._sample_id = None
        self._sample_at = None
        self._sample_rear = None
        self._reboot_count = None
        self._rear = None
        self._start_rear = None
        self._best_rear = None
        self._progress_reference = None
        self._progress_at = None
        self._clear_stable()
        self._result = self._status("park_ready")
        return self.snapshot()

    def snapshot(self):
        """回傳獨立狀態副本，顯示端不能更改內部決策。"""
        return dict(self._result)

    def _status(self, reason, esc=NEUTRAL):
        progress = (0.0 if self._start_rear is None or self._rear is None
                    else self._start_rear - self._rear)
        return {"state": self._state, "reason": reason,
                "esc": esc, "servo": NEUTRAL, "stop": self._stopped,
                "completed": self._state == "complete", "rear_mm": self._rear,
                "target_mm": self.config["target_rear_mm"], "progress_mm": progress}

    def _publish(self, reason, esc=NEUTRAL):
        self._result = self._status(reason, esc)
        return self.snapshot()

    def _abort(self, reason):
        self._state, self._stopped = "aborted", True
        return self._publish(reason)

    def _clear_stable(self):
        self._stable_at = None
        self._stable_min = None
        self._stable_max = None
        self._stable_count = 0

    def _stable(self, rear, stamp, duration):
        """只能在新測距樣本且 M4 回報中立時呼叫。"""
        if (self._stable_at is None
                or max(self._stable_max, rear) - min(self._stable_min, rear)
                > self.config["stable_span_mm"]):
            self._stable_at = stamp
            self._stable_min = self._stable_max = rear
            self._stable_count = 1
        else:
            self._stable_min = min(self._stable_min, rear)
            self._stable_max = max(self._stable_max, rear)
            self._stable_count += 1
        return (self._stable_count >= self.config["min_samples"]
                and stamp - self._stable_at >= duration)

    @staticmethod
    def _distances(health):
        if not isinstance(health, Mapping) or not isinstance(health.get("tof"), Mapping):
            return None
        distances = {}
        for direction in DIRECTIONS:
            sample = health["tof"].get(direction)
            if not isinstance(sample, Mapping) or sample.get("valid") is not True:
                return None
            value = sample.get("distance_mm")
            if not _number(value) or value <= 0:
                return None
            distances[direction] = float(value)
        return distances

    def step(self, health, now=None):
        """一次決策。stop=True 時上層應退出停車模式並維持中立。

        到位需要多筆新的 MCU 測距時間戳及持續的中立輸出回報。
        重複封包、單純重讀狀態或經過一段時間，都不算新量測。
        """
        if self._stopped:
            return self.snapshot()
        stamp = self._clock() if now is None else now
        if not _number(stamp) or stamp < self._last_at:
            return self._abort("park_clock_invalid")
        stamp = float(stamp)
        self._last_at = stamp
        distance = self._distances(health)
        if distance is None:
            return self._abort("park_sensor_invalid")
        rear = distance["rear"]
        self._rear = rear
        cfg = self.config
        if any(value <= cfg["clearance_mm"] for value in distance.values()):
            return self._abort("park_path_blocked")
        m4 = health.get("m4")
        if not isinstance(m4, Mapping) or not _integer(m4.get("reboot_count")):
            return self._abort("park_sample_invalid")
        reboot = m4["reboot_count"]
        if self._reboot_count is None:
            self._reboot_count = reboot
        elif reboot != self._reboot_count:
            return self._abort("park_m4_reboot")
        token = health["tof"]["rear"].get("sample_uptime_ms")
        if not _integer(token, 0, 0xffffffff):
            return self._abort("park_sample_invalid")
        is_new = self._sample_id is None
        if not is_new:
            advance = (token - self._sample_id) & 0xffffffff
            if advance >= 0x80000000:
                return self._abort("park_sample_regression")
            is_new = advance != 0
            if not is_new and rear != self._sample_rear:
                return self._abort("park_sample_invalid")
        if self._sample_at is not None and stamp - self._sample_at > MAX_SAMPLE_GAP_S + 1e-9:
            self._clear_stable()
        if is_new:
            if self._sample_rear is not None and abs(rear - self._sample_rear) > cfg["max_step_mm"]:
                return self._abort("park_distance_jump")
            self._sample_id, self._sample_at, self._sample_rear = token, stamp, rear
        neutral = (m4.get("output_esc") == NEUTRAL
                   and m4.get("output_servo") == NEUTRAL)
        if not neutral:
            self._clear_stable()

        if self._state == "idle":
            self._state, self._state_at = "preparing", stamp

        if self._state == "preparing":
            if not cfg["start_min_mm"] <= rear <= cfg["start_max_mm"]:
                return self._abort("park_start_out_of_range")
            if stamp - self._state_at >= cfg["prepare_timeout_s"]:
                return self._abort("park_prepare_timeout")
            if is_new and neutral and self._stable(rear, stamp, cfg["prepare_s"]):
                self._state, self._state_at = "reversing", stamp
                self._start_rear = self._best_rear = self._progress_reference = rear
                self._progress_at = stamp
                self._clear_stable()
                return self._publish("park_reversing", cfg["reverse_esc"])
            return self._publish("park_preparing")

        if self._state == "reversing":
            if stamp - self._state_at >= cfg["max_reverse_s"]:
                return self._abort("park_timeout")
            # 逾期的新資料不能回頭延長已到期的無進展期限，抵達目標亦同。
            if stamp - self._progress_at >= cfg["no_progress_s"]:
                return self._abort("park_no_progress")
            if is_new:
                if rear - self._best_rear >= cfg["wrong_way_mm"]:
                    return self._abort("park_wrong_direction")
                self._best_rear = min(self._best_rear, rear)
                if self._progress_reference - self._best_rear >= cfg["min_progress_mm"]:
                    self._progress_reference = self._best_rear
                    self._progress_at = stamp
                if rear <= cfg["target_rear_mm"]:
                    self._state, self._state_at = "settling", stamp
                    self._clear_stable()
                else:
                    return self._publish("park_reversing", cfg["reverse_esc"])
            else:
                return self._publish("park_reversing", cfg["reverse_esc"])

        # 首次抵達目標後只會中立／完成／中止，不會再次發出倒車命令。
        if not cfg["arrival_min_mm"] <= rear <= cfg["arrival_max_mm"]:
            return self._abort("park_target_out_of_range")
        if stamp - self._state_at >= cfg["settle_timeout_s"]:
            return self._abort("park_settle_timeout")
        if is_new and neutral and self._stable(rear, stamp, cfg["settle_s"]):
            self._state, self._stopped = "complete", True
            return self._publish("park_complete")
        return self._publish("park_settling")
