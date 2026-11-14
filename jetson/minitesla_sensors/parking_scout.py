"""巡邏中辨識車位與停穩確認；只回傳感測結果，絕不產生控制命令。"""
import math
import time

from .parking_auto import AutomaticParkingEngine, extract_scan, finite, wrap_angle


class ParkingScout(AutomaticParkingEngine):
    """沿用停車的來源去重、車位座標及感測檢查，所有計算留在子程序。"""
    def __init__(self, config, clock=time.monotonic):
        super().__init__(config, clock=clock)

    def reset(self):
        super().reset()
        self.reason = "mission_searching"
        self.rearmed = True
        self.ignore_until = 0.0
        self.clear_started = None
        self.clear_last = None
        self.ready_pose = None

    def _report(self, message, reason=None, **unused):
        # 來源只能是 extract_scan 的最舊回波時間；沒有可用回波時使用
        # 0 並回 aborted。父端不得把錯誤報告當作新鮮偵測或移動許可。
        report = {"kind": "mission", "session": self.key[0], "epoch": self.key[1],
                  "input_at": message["at"], "created_at": self.clock(),
                  "source_at": self.last_source_at if self.last_source_at is not None else 0.0,
                  "state": self.state, "reason": reason or self.reason,
                  "pose": list(self.pose) if self.pose is not None else None,
                  "rearmed": self.rearmed}
        if self.fit_error is not None:
            report["fit_error_m"] = self.fit_error
        return report

    def _failed(self, message, reason, state="aborted"):
        self.state, self.reason = state, reason
        return self._report(message)

    @staticmethod
    def _valid_pose(pose):
        return (isinstance(pose, (list, tuple)) and len(pose) == 3
                and all(finite(value) for value in pose))

    def _ready_unchanged(self, snapshot):
        return (self._neutral(snapshot) and self.ready_pose is not None
                and math.hypot(self.pose[0] - self.ready_pose[0],
                               self.pose[1] - self.ready_pose[1]) <= .025
                and abs(wrap_angle(self.pose[2] - self.ready_pose[2])) <= math.radians(2))

    def step(self, message):
        from .parking_geometry import detect_bays, track_bay
        if not isinstance(message, dict):
            return None
        control = message.get("control", {})
        if not isinstance(control, dict):
            return None
        epoch, session, at = control.get("epoch"), message.get("session"), message.get("at")
        now, mode = self.clock(), control.get("mode")
        if (not isinstance(session, str) or not isinstance(epoch, int) or isinstance(epoch, bool)
                or not finite(at) or not 0 <= at <= now):
            return None
        if mode not in ("search", "offer"):
            self.reset()
            return None
        key = (session, epoch, mode)
        if self.key != key:
            self.reset()
            self.key = key
            self.started = self.state_at = now
            mission = control.get("mission", {})
            if not isinstance(mission, dict):
                return self._failed(message, "mission_invalid")
            if mode == "offer":
                pose = mission.get("pose")
                if not self._valid_pose(pose):
                    return self._failed(message, "mission_target_missing")
                self.pose = list(map(float, pose))
                self.state, self.reason = "verifying", "mission_verifying"
            else:
                self.state, self.reason = "searching", "mission_searching"
                self.rearmed = mission.get("require_clear") is not True
                if not self.rearmed:
                    until = mission.get("skip_until")
                    if not finite(until) or until < 0:
                        return self._failed(message, "mission_invalid")
                    self.ignore_until = until
        if self.state in ("aborted", "lost"):
            return self._report(message)
        if now - at > self.cfg["proposal_timeout_s"]:
            return None
        if (mode == "offer" and self.state != "ready"
                and now - self.started >= self.cfg["mission_offer_acquire_s"]):
            return self._failed(message, "mission_acquire_timeout")
        snapshot = message.get("snapshot", {})
        if not isinstance(snapshot, dict):
            return self._failed(message, "mission_sensor_invalid")
        problem = self._sensor_reason(snapshot)
        if problem:
            return self._failed(message, problem.replace("park_", "mission_", 1))
        # 即使掃描未更新，也不能把非中立硬體當成已停穩的通知狀態。
        if self.state == "ready" and not self._neutral(snapshot):
            return self._failed(message, "mission_vehicle_moved", "lost")
        scan = extract_scan(snapshot, self.cfg, now)
        if scan is None:
            return self._failed(message, "mission_lidar_unavailable")
        if not self._new_scan(scan):
            return None
        points, observed = scan["points"], scan["observation_at"]
        if mode == "search":
            candidates = detect_bays(points, self.cfg)
            if not self.rearmed:
                # N 後十秒內禁止重複通知，並必須真正離開可見車位一秒。
                # 有回波中斷時重新計算連續無車位時間，不能靠等待舊資料通過。
                if candidates:
                    self.clear_started = self.clear_last = None
                else:
                    if (self.clear_started is None or self.clear_last is None
                            or observed - self.clear_last > self.cfg["mission_report_timeout_s"]):
                        self.clear_started = observed
                    self.clear_last = observed
                    if (now >= self.ignore_until
                            and observed - self.clear_started >= self.cfg["mission_clear_s"]):
                        self.rearmed = True
                self.pose, self.fit_error = None, None
                self.state = "searching"
                self.reason = "mission_searching" if self.rearmed else "mission_skipping"
                return self._report(message)
            if not candidates:
                self.pose, self.fit_error = None, None
                self.state, self.reason = "searching", "mission_searching"
                return self._report(message)
            candidates.sort(key=lambda c: (math.hypot(c["pose"][0] - self.cfg["bay_depth_m"], c["pose"][1]),
                                           c["pose"][0], c["pose"][1]))
            candidate = candidates[0]
            if not self._valid_pose(candidate.get("pose")):
                return self._failed(message, "mission_pose_lost")
            self.pose = list(map(float, candidate["pose"]))
            self.fit_error = float(candidate.get("fit_error_m", 0.0))
            self.state, self.reason = "found", "mission_found"
            return self._report(message)
        # offer 整段只能追蹤先前選定的候選；不再呼叫 detect_bays 換位。
        candidate = track_bay(points, self.cfg, self.pose)
        if candidate is None:
            return self._failed(message, "mission_pose_lost", "lost")
        problem = self._accept_pose(candidate, observed)
        if problem:
            return self._failed(message, problem.replace("park_", "mission_", 1), "lost")
        if self.state == "ready":
            if not self._ready_unchanged(snapshot):
                return self._failed(message, "mission_vehicle_moved", "lost")
            return self._report(message)
        if (self._stable(snapshot, observed, max(.5, self.cfg["prepare_s"]))
                and self.stable_count >= 4):
            self.ready_pose = list(self.pose)
            self.state, self.reason = "ready", "mission_ready"
        return self._report(message)
