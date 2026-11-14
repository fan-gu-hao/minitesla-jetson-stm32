"""四向測距的反應式巡邏；只產生命令，不直接操作裝置。

這不是建圖或定點導航。前方接近障礙時選擇較空的一側並保持轉向；
四向距離不足、感測無效或轉向逾時，立即要求控制權管理器回到待機。
停車結果會保持到 reset()，不會因障礙物移開自行恢復行駛。
呼叫者必須在同一把控制鎖內呼叫本類別，並先檢查資料時效及 M4 狀態。
"""

import math
import time
from collections.abc import Mapping


NEUTRAL = 152
DIRECTIONS = ("front", "rear", "left", "right")


def _number(value):
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return False
    try:
        return math.isfinite(value)
    except (OverflowError, ValueError):
        return False


class ReactivePatrol:
    """同步執行的巡邏決策器；設定讀取完整設定檔的 autonomy 區段。"""

    DEFAULTS = {
        "forward_esc": 154,
        "left_servo": 128,
        "right_servo": 176,
        "stop_front_mm": 500,
        "side_stop_mm": 450,
        "turn_start_mm": 950,
        "clear_front_mm": 1100,
        "side_clear_mm": 650,
        "max_turn_s": 2.0,
        "stable_clear_s": 0.5,
    }

    def __init__(self, config=None, clock=time.monotonic):
        if config is None:
            config = {}
        if not isinstance(config, Mapping):
            raise ValueError("巡邏設定必須是物件")
        section = config.get("autonomy", {})
        if not isinstance(section, Mapping):
            raise ValueError("autonomy 必須是設定物件")
        self.config = {key: section.get(key, default)
                       for key, default in self.DEFAULTS.items()}
        self._validate_config()
        if not callable(clock):
            raise ValueError("巡邏時鐘必須可呼叫")
        self._clock = clock
        self.reset()

    def _validate_config(self):
        cfg = self.config
        for name, value in cfg.items():
            if not _number(value):
                raise ValueError("autonomy.%s 必須是有限數值" % name)
        for name in ("forward_esc", "left_servo", "right_servo"):
            if not isinstance(cfg[name], int):
                raise ValueError("autonomy.%s 必須是整數" % name)
        if not 153 <= cfg["forward_esc"] <= 156:
            raise ValueError("autonomy.forward_esc 必須介於 153 與 156")
        if not 94 <= cfg["left_servo"] < NEUTRAL:
            raise ValueError("autonomy.left_servo 必須介於 94 與 151")
        if not NEUTRAL < cfg["right_servo"] <= 210:
            raise ValueError("autonomy.right_servo 必須介於 153 與 210")
        # 不允許把本層的停車界線設在既有 M4 解除保護距離以下。
        if cfg["stop_front_mm"] < 400 or cfg["side_stop_mm"] < 400:
            raise ValueError("巡邏停車距離不得低於 400 毫米")
        if not (cfg["stop_front_mm"] < cfg["turn_start_mm"]
                < cfg["clear_front_mm"]):
            raise ValueError("前方門檻必須符合：停車 < 開始轉向 < 恢復直行")
        if cfg["side_clear_mm"] <= cfg["side_stop_mm"]:
            raise ValueError("側面轉向淨空必須大於側面停車距離")
        if not 0 < cfg["stable_clear_s"] < cfg["max_turn_s"]:
            raise ValueError("時間必須符合：0 < 穩定淨空時間 < 最長轉向時間")

    def reset(self, now=None):
        """清除先前巡邏及停車狀態；本身不產生油門。"""
        stamp = self._clock() if now is None else now
        if not _number(stamp) or stamp < 0:
            raise ValueError("巡邏時鐘必須是非負有限數值")
        self._last_at = float(stamp)
        self._turn_direction = None
        self._turn_started_at = None
        self._clear_since = None
        self._stopped = False
        self._result = {"state": "idle", "reason": "patrol_ready",
                        "esc": NEUTRAL, "servo": NEUTRAL, "stop": False}
        return self.snapshot()

    def snapshot(self):
        """回傳副本，避免顯示端改到控制結果。"""
        return dict(self._result)

    def _stop(self, reason):
        self._stopped = True
        self._result = {"state": "stopped", "reason": reason,
                        "esc": NEUTRAL, "servo": NEUTRAL, "stop": True}
        return self.snapshot()

    def _move(self, direction=None):
        state = "cruise" if direction is None else "turn_" + direction
        servo = NEUTRAL if direction is None else self.config[direction + "_servo"]
        self._result = {"state": state, "reason": "patrol_" + state,
                        "esc": self.config["forward_esc"], "servo": servo,
                        "stop": False}
        return self.snapshot()

    @staticmethod
    def _distances(health):
        if not isinstance(health, Mapping):
            return None
        tof = health.get("tof")
        if not isinstance(tof, Mapping):
            return None
        distances = {}
        for direction in DIRECTIONS:
            sample = tof.get(direction)
            if not isinstance(sample, Mapping) or sample.get("valid") is not True:
                return None
            value = sample.get("distance_mm")
            if not _number(value) or value <= 0:
                return None
            distances[direction] = float(value)
        return distances

    def step(self, health, now=None):
        """回傳 state、reason、esc、servo、stop；stop=True 要求離開巡邏。

        valid 必須由感測整合層在檢查時間戳後產生。本類別另檢查四向
        數值完整性；不把無效、缺值或無限大解讀成前方沒有障礙物。
        """
        if self._stopped:
            return self.snapshot()
        stamp = self._clock() if now is None else now
        if not _number(stamp) or stamp < self._last_at:
            return self._stop("patrol_clock_invalid")
        stamp = float(stamp)
        self._last_at = stamp
        distance = self._distances(health)
        if distance is None:
            return self._stop("patrol_sensor_invalid")
        cfg = self.config
        if (distance["front"] <= cfg["stop_front_mm"]
                or any(distance[d] <= cfg["side_stop_mm"]
                       for d in ("rear", "left", "right"))):
            return self._stop("patrol_path_blocked")
        # 兩側同時過近時不嘗試穿越狹窄通道；沒有倒車或原地轉向動作。
        if (distance["left"] < cfg["side_clear_mm"]
                and distance["right"] < cfg["side_clear_mm"]):
            return self._stop("patrol_path_blocked")

        if self._turn_direction is not None:
            if stamp - self._turn_started_at >= cfg["max_turn_s"]:
                return self._stop("patrol_turn_timeout")
            # 已選的一側變得不足時停車，不在左右兩側間反覆切換。
            if distance[self._turn_direction] < cfg["side_clear_mm"]:
                return self._stop("patrol_path_blocked")
            if distance["front"] >= cfg["clear_front_mm"]:
                if self._clear_since is None:
                    self._clear_since = stamp
                elif stamp - self._clear_since >= cfg["stable_clear_s"]:
                    self._turn_direction = None
                    self._turn_started_at = None
                    self._clear_since = None
                    return self._move()
            else:
                self._clear_since = None
            return self._move(self._turn_direction)

        if distance["front"] < cfg["turn_start_mm"]:
            # 相同淨空時固定向左，確保決策可重現；轉向後保持方向。
            direction = ("left" if distance["left"] >= distance["right"] else "right")
            if distance[direction] < cfg["side_clear_mm"]:
                return self._stop("patrol_path_blocked")
            self._turn_direction = direction
            self._turn_started_at = stamp
            self._clear_since = None
            return self._move(direction)
        return self._move()
