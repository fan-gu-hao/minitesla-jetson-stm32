"""第十階段：巡邏找位、限時確認與隔離停車規劃程序，控制權及最終 UART 守門仍由本模組持有。

模式切換與 UART 送出仍依同一把鎖排序；新模式先中立，再收本代次心跳。
找位任務停穩後通知；只有本次通知的明確同意能進入停車，M4 保護永遠優先。
"""
from collections import OrderedDict
import copy
import math
import secrets
import threading
import time

from .autonomy import ReactivePatrol
from .parking import StraightReverseParking

NEUTRAL = (152, 152)
ESC_MIN, ESC_MAX = 142, 160
SERVO_MIN, SERVO_MAX = 94, 210
DIRECTIONS = ("front", "rear", "left", "right")


def _integer(value, low=0, high=(1 << 63) - 1):
    return isinstance(value, int) and not isinstance(value, bool) and low <= value <= high


def _finite(value, low=0.0, high=1e12):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value) and low <= value <= high)


AUTO_STATES = ("searching", "acquiring", "planning", "aligning", "reversing",
               "gear_wait", "settling", "complete", "aborted")
AUTO_SOURCE_TTL = 0.35
AUTO_WAIT_TIMEOUT = 5.0
ACTIVE_MODES = ("manual", "patrol", "park", "search", "offer")
MISSION_STATES = ("searching", "found", "verifying", "ready", "lost", "aborted")


class ControlAuthority:
    """唯一的上層控制輸出來源。所有狀態讀寫受同一把鎖保護。

    session 隔開不同程式執行期，seq 排除舊網路封包，epoch 隔開模式切換前後。
    action.id 用於重複請求去重；同一請求重送不能重複切換或自動解除故障。
    這些欄位提供命令順序管理，不宣稱提供加密或網路身分驗證。
    """

    def __init__(self, config=None, clock=time.monotonic, session=None):
        settings = (config or {}).get("control", {})
        self.timeout = settings.get("heartbeat_timeout_s", 0.3)
        self.handoff = settings.get("handoff_neutral_s", 0.1)
        self.video_timeout = settings.get("video_timeout_s", 1.0)
        for key, value in (("heartbeat_timeout_s", self.timeout), ("handoff_neutral_s", self.handoff)):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError("control.%s must be finite" % key)
        if not 0.05 <= self.timeout <= 0.3:
            raise ValueError("control.heartbeat_timeout_s must be in [0.05,0.3]")
        if not 0.05 <= self.handoff < self.timeout:
            raise ValueError("control.handoff_neutral_s must be >=0.05 and < heartbeat_timeout_s")
        if (isinstance(self.video_timeout, bool) or not isinstance(self.video_timeout, (int, float))
                or not math.isfinite(self.video_timeout) or not 0.2 <= self.video_timeout <= 2.0):
            raise ValueError("control.video_timeout_s must be in [0.2,2.0]")
        self._camera_timeout_ms = (config or {}).get("camera", {}).get("stale_ms", 1000)
        if (isinstance(self._camera_timeout_ms, bool)
                or not isinstance(self._camera_timeout_ms, (int, float))
                or not math.isfinite(self._camera_timeout_ms) or self._camera_timeout_ms <= 0):
            raise ValueError("camera.stale_ms 必須為有限正數")
        self._planner = ReactivePatrol(config, clock=clock)
        self._parking = StraightReverseParking(config, clock=clock)
        self._parking_status = {"state": "idle", "reason": "startup", "completed": False}
        self._auto_present = "auto_parking" in (config or {})
        self._auto_config = None
        if self._auto_present:
            from .parking_config import validate_settings
            self._auto_config = validate_settings((config or {})["auto_parking"])
        self._auto_enabled = bool(self._auto_config and self._auto_config.get("enabled") is True)
        self._auto_proposal = None
        self._auto_source_high = None
        self._auto_plan_started = None
        self._auto_moved = False
        self._auto_reboot = None
        if self._auto_present and not self._auto_enabled:
            self._parking_status["reason"] = "auto_park_not_configured"
        self._last_park = None
        self._mission_status = {"active": False, "state": "idle", "reason": "startup",
                                "offer_id": None, "pose": None, "approved_pose": None,
                                "skip_until": 0.0, "require_clear": False}
        self._mission_started = self._mission_deadline = None
        self._mission_confirm_deadline = None
        self._mission_report = self._mission_source_high = None
        self._last_mission = self._mission_reboot = None
        self._mission_neutral_since = None
        self._mission_offer_samples = 0
        self._mission_m4_neutral = False
        self._mission_health_at = None
        self._patrol_status = {"state": "idle", "reason": "startup"}
        self._last_patrol = None
        self._perception = {"valid": False, "age_ms": None, "captured_at": None,
                            "person_count": None, "person_present": None,
                            "event_count": 0, "last_event_at": None}
        self._clock = clock
        self._lock = threading.RLock()
        self.session = session or secrets.token_hex(16)
        self._mode, self._reason, self._epoch = "standby", "startup", 0
        self._last_seq = -1
        self._last_rx = self._last_drive = self._entered = None
        self._drive = NEUTRAL
        self._output = NEUTRAL
        self._neutral_seen = False
        self._last_action = None
        self._action_cache = OrderedDict()
        self._action_high = -1
        self._rejected = 0
        self._transitions = 0
        self._last_transition = None
        self._video_state = "waiting"
        self._video_at = self._video_ack = self._video_frame = None
        self._video_seq = None
        self._video_losses = 0

    def _video_reason(self, now):
        if self._video_state == "stopped":
            return "video_sender_stopped"
        if self._video_state == "disconnected":
            return "video_disconnected"
        if self._video_ack is None or self._video_state != "confirmed":
            return "video_unconfirmed"
        if now - self._video_ack > self.video_timeout:
            return "video_ack_timeout"
        if now - self._video_frame > self.video_timeout:
            return "video_frame_stale"
        return None

    def update_video(self, report, now=None):
        """接收本機傳送程序事件，使用事件原時間，不把延遲事件當新影格。

        losses 是傳送程序累積斷線數，讓有界佇列丟掉單筆斷線事件時，
        後續成功事件仍能保留失聯證據。恢復報告永不自動清除故障。
        """
        now = self._now(now)
        with self._lock:
            if not isinstance(report, dict):
                return False
            at, state, losses = report.get("at"), report.get("state"), report.get("losses")
            if (isinstance(at, bool) or not isinstance(at, (int, float))
                    or not math.isfinite(at) or not 0 <= at <= now
                    or state not in ("connected", "confirmed", "disconnected", "stopped")
                    or not _integer(losses) or losses < self._video_losses
                    or (self._video_at is not None and at <= self._video_at)):
                return False
            if state == "confirmed":
                frame, sequence = report.get("captured_at"), report.get("frame_seq")
                if (isinstance(frame, bool) or not isinstance(frame, (int, float))
                        or not math.isfinite(frame) or not 0 <= frame <= at
                        or not _integer(sequence, 1)
                        or (self._video_seq is not None and sequence <= self._video_seq)):
                    return False
            # 新成功回報到達前已經逾時，也必須先鎖定，不能悄悄續駛。
            reason = self._video_reason(now)
            if self._mode in ACTIVE_MODES and (reason or losses > self._video_losses):
                self._transition("fault", reason or "video_disconnected", now)
            self._video_at, self._video_state, self._video_losses = at, state, losses
            if state == "confirmed":
                self._video_ack, self._video_frame, self._video_seq = at, frame, sequence
            reason = self._video_reason(now)
            if self._mode in ACTIVE_MODES and reason:
                self._transition("fault", reason, now)
            return True

    def _readiness_reason(self, health, now):
        return self._health_reason(health) or self._video_reason(now)

    def _observe_perception(self, health):
        summary = health.get("perception") if isinstance(health, dict) else None
        if isinstance(summary, dict):
            self._perception = {key: copy.deepcopy(summary.get(key))
                                for key in self._perception}
        else:
            self._perception.update(valid=False, age_ms=None, captured_at=None,
                                    person_count=None, person_present=None)

    def _perception_snapshot(self, now):
        summary = dict(self._perception)
        captured = summary.get("captured_at")
        timestamp_ok = (isinstance(captured, (int, float)) and not isinstance(captured, bool)
                        and math.isfinite(captured) and 0 <= captured <= now)
        age = (now - captured) * 1000 if timestamp_ok else None
        summary["age_ms"] = age
        summary["valid"] = bool(summary.get("valid") is True and age is not None
                                and age <= self._camera_timeout_ms)
        if not summary["valid"]:
            summary["person_count"] = summary["person_present"] = None
        return summary

    def _patrol_camera_reason(self, now):
        return None if self._perception_snapshot(now)["valid"] else "patrol_camera_unavailable"

    def _parking_camera_reason(self, now):
        return None if self._perception_snapshot(now)["valid"] else "park_camera_unavailable"

    def _video_snapshot(self, now):
        reason = self._video_reason(now)
        return {"state": self._video_state, "ready": reason is None, "reason": reason,
                "frame_age_ms": None if self._video_frame is None else max(0, now-self._video_frame)*1000,
                "ack_age_ms": None if self._video_ack is None else max(0, now-self._video_ack)*1000,
                "frame_seq": self._video_seq, "disconnect_count": self._video_losses,
                "timeout_ms": self.video_timeout*1000}

    def _now(self, supplied):
        value = self._clock() if supplied is None else supplied
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError("invalid monotonic timestamp")
        return float(value)

    @staticmethod
    def _health_reason(health):
        if not isinstance(health, dict):
            return "m4_offline"
        m4, tof = health.get("m4", {}), health.get("tof", {})
        if not m4.get("connected"):
            return "m4_offline"
        for direction in DIRECTIONS:
            if not tof.get(direction, {}).get("valid"):
                return "tof_invalid_" + direction
        if m4.get("mode") == 0:
            return "m4_guard_latched"
        if m4.get("mode") not in (1, 2):
            return "m4_state_unknown"
        return None

    def _transition(self, mode, reason, now, preserve_mission=False):
        previous = self._mode
        self._mode, self._reason = mode, reason
        self._epoch += 1
        self._transitions += 1
        self._drive = self._output = NEUTRAL
        self._neutral_seen = False
        self._last_drive = None
        self._last_patrol = self._last_park = None
        self._auto_proposal = None
        self._auto_source_high = None
        self._auto_plan_started = None
        self._auto_moved = False
        self._auto_reboot = None
        self._entered = now if mode in ACTIVE_MODES else None
        self._last_mission = None
        self._mission_report = self._mission_source_high = None
        self._mission_neutral_since = None
        self._mission_offer_samples = 0
        self._mission_confirm_deadline = None
        if not preserve_mission:
            if self._mission_status["active"]:
                self._mission_status.update(active=False, state="aborted", reason=reason,
                                            pose=None, approved_pose=None, offer_id=None,
                                            skip_until=0.0, require_clear=False)
                self._mission_status.pop("fit_error_m", None)
            self._mission_started = self._mission_deadline = self._mission_reboot = None
        self._planner.reset(now=now)
        self._parking.reset(now=now)
        self._parking_status = {"state": "handoff" if mode == "park" else "idle",
                                "reason": reason, "completed": False}
        self._patrol_status = {"state": "handoff" if mode in ("patrol", "search") else "idle", "reason": reason}
        self._last_transition = {"from": previous, "to": mode, "reason": reason,
                                 "monotonic_s": now, "epoch": self._epoch}

    def _abort_auto(self, reason, now):
        self._transition("standby", reason, now)
        self._parking_status = {"state": "aborted", "reason": reason,
                                "completed": False, "stop": True,
                                "esc": NEUTRAL[0], "servo": NEUTRAL[1]}

    def _mission_setting(self, name, fallback):
        return (self._auto_config or {}).get(name, fallback)

    def _start_mission(self, health, now):
        self._mission_started = now
        self._mission_deadline = now + self._mission_setting("mission_search_timeout_s", 120.0)
        self._mission_reboot = health["m4"]["reboot_count"]
        self._mission_status = {"active": True, "state": "searching", "reason": "mission_searching",
                                "offer_id": None, "pose": None, "approved_pose": None,
                                "skip_until": 0.0, "require_clear": False}

    def _mission_health_reason(self, health, starting=False):
        reboot = health.get("m4", {}).get("reboot_count")
        if not _integer(reboot) or (not starting and reboot != self._mission_reboot):
            return "mission_m4_reboot"
        for direction in DIRECTIONS:
            sensor = health.get("tof", {}).get(direction, {})
            if sensor.get("valid") is not True or not _finite(sensor.get("distance_mm"), 0, 65535):
                return "mission_tof_invalid_" + direction
            if sensor["distance_mm"] <= max(450, self._auto_config["tof_clearance_mm"]):
                return "mission_path_blocked"
        return None

    def _abort_mission(self, reason, now):
        self._transition("standby", reason, now)

    def _mission_expiry_reason(self, now):
        # 所有期限用 >=，截止的同一時刻也不能同意；重送/略過不能延長任務。
        if self._mission_deadline is None or now >= self._mission_deadline:
            return "mission_search_timeout"
        if (self._mission_confirm_deadline is not None
                and now >= self._mission_confirm_deadline):
            return "mission_decision_timeout"
        if (self._mode == "offer" and self._mission_confirm_deadline is None
                and now - self._entered >= self._mission_setting("mission_offer_acquire_s", 5.0)):
            return "mission_verify_timeout"
        if self._mission_report is None:
            if now - self._entered >= AUTO_WAIT_TIMEOUT:
                return "mission_worker_timeout"
        else:
            timeout = self._mission_setting("mission_report_timeout_s", 0.5)
            if (now - self._mission_report["source_at"] > timeout
                    or now - self._mission_report["input_at"] > timeout):
                return "mission_source_stale"
        return None

    def _mission_snapshot(self, now):
        result = copy.deepcopy(self._mission_status)
        result["decision_s"] = self._mission_setting("mission_confirm_s", 15.0)
        deadline = self._mission_confirm_deadline
        result["remaining_s"] = (None if deadline is None else
                                 max(0.0, min(deadline, self._mission_deadline) - now))
        return result

    def update_mission(self, report, now=None):
        """子程序僅提出車位；這裡鎖定控制權與期限，不執行幾何或 UART I/O。"""
        with self._lock:
            now = self._now(now)
            if (not self._auto_enabled or self._mode not in ("search", "offer")
                    or not isinstance(report, dict) or report.get("kind") != "mission"
                    or report.get("session") != self.session or report.get("epoch") != self._epoch):
                return False
            reason = self._mission_expiry_reason(now)
            if reason:
                self._abort_mission(reason, now)
                return False
            stamps = [report.get(key) for key in ("source_at", "input_at", "created_at")]
            pose = report.get("pose")
            valid_pose = (isinstance(pose, (list, tuple)) and len(pose) == 3
                          and all(_finite(v, -10000, 10000) for v in pose))
            state = report.get("state")
            valid = (all(_finite(v) for v in stamps) and stamps[0] <= stamps[1] <= stamps[2] <= now
                     and stamps[1] >= self._entered and _integer(report.get("epoch"))
                     and state in MISSION_STATES and type(report.get("rearmed")) is bool
                     and isinstance(report.get("reason"), str) and 0 < len(report["reason"]) <= 80
                     and (pose is None or valid_pose))
            if state in ("found", "verifying", "ready") and not valid_pose:
                valid = False
            if "fit_error_m" in report and not _finite(report["fit_error_m"], 0, 10000):
                valid = False
            if not valid:
                self._abort_mission("mission_report_invalid", now)
                return False
            # 當前程序的失效/失位訊號永遠先停止，不能因同一來源而被去重吞掉。
            if state in ("lost", "aborted"):
                self._abort_mission(report["reason"], now)
                return True
            ttl = min(AUTO_SOURCE_TTL, self._auto_config["proposal_timeout_s"])
            if now - stamps[0] > ttl or now - stamps[1] > ttl:
                self._abort_mission("mission_source_stale", now)
                return False
            previous = self._mission_report
            if ((previous is not None and stamps[1] <= previous["input_at"])
                    or (self._mission_source_high is not None and stamps[0] <= self._mission_source_high)):
                return False
            if ((self._mode == "search" and state not in ("searching", "found"))
                    or (self._mode == "offer" and state not in ("verifying", "ready"))):
                self._abort_mission("mission_report_invalid", now)
                return False
            if (self._mode == "offer" and self._mission_status["state"] == "awaiting"
                    and state != "ready"):
                self._abort_mission("mission_target_lost", now)
                return False
            self._mission_report = copy.deepcopy(report)
            self._mission_source_high = stamps[0]
            if self._mode == "search":
                if (self._mission_status["require_clear"] and report["rearmed"]
                        and now >= self._mission_status["skip_until"]):
                    self._mission_status.update(require_clear=False, state="searching",
                                                reason="mission_searching")
                if state == "found" and not self._mission_status["require_clear"]:
                    self._transition("offer", "mission_candidate_found", now, preserve_mission=True)
                    self._mission_status.update(state="verifying", reason="mission_verifying",
                                                offer_id=self._epoch, pose=list(pose))
                    if "fit_error_m" in report:
                        self._mission_status["fit_error_m"] = report["fit_error_m"]
                return True
            self._mission_status["pose"] = list(pose)
            if "fit_error_m" in report:
                self._mission_status["fit_error_m"] = report["fit_error_m"]
            if stamps[0] >= self._entered:
                self._mission_offer_samples += 1
            # 子程序與父程序雙邊確認中立；4個新掃描及固定穩定時間後才發出通知。
            neutral = (self._mission_m4_neutral and self._mission_health_at is not None
                       and now - self._mission_health_at <= self.timeout
                       and self._mission_neutral_since is not None
                       and now - self._mission_neutral_since >= 0.5)
            if (state == "ready" and neutral and self._mission_offer_samples >= 4
                    and self._neutral_seen and self._last_mission is not None
                    and now - self._last_mission <= self.timeout):
                if self._mission_confirm_deadline is None:
                    self._mission_confirm_deadline = now + self._mission_setting("mission_confirm_s", 15.0)
                self._mission_status.update(state="awaiting", reason="mission_awaiting_decision")
            return True

    def _decide_mission(self, action, packet, health, now):
        if (self._mode != "offer" or self._mission_status["state"] != "awaiting"
                or self._mission_confirm_deadline is None):
            return False, "mission_no_offer"
        if (not _integer(action.get("offer_id"), 1)
                or action["offer_id"] != self._mission_status["offer_id"]):
            return False, "mission_offer_mismatch"
        if (packet["esc"], packet["servo"]) != NEUTRAL:
            return False, "neutral_required"
        reason = self._mission_expiry_reason(now)
        if reason:
            self._abort_mission(reason, now)
            return False, reason
        if action["name"] == "park_decline":
            self._transition("search", "mission_declined", now, preserve_mission=True)
            self._mission_status.update(state="skipping", reason="mission_declined", offer_id=None,
                                        pose=None, approved_pose=None, require_clear=True,
                                        skip_until=now + self._mission_setting("mission_ignore_s", 10.0))
            self._mission_status.pop("fit_error_m", None)
            return True, "mission_declined"
        report = self._mission_report
        ttl = min(AUTO_SOURCE_TTL, self._auto_config["proposal_timeout_s"])
        reason = self._readiness_reason(health, now) or self._patrol_camera_reason(now)
        reason = reason or self._mission_health_reason(health)
        if reason is None and (not report or report["state"] != "ready"
                               or now - report["source_at"] > ttl or now - report["input_at"] > ttl):
            reason = "mission_source_stale"
        if reason is None and (not self._mission_m4_neutral or not self._neutral_seen
                               or self._mission_neutral_since is None
                               or now - self._mission_neutral_since < 0.5):
            reason = "mission_not_stationary"
        if reason:
            self._abort_mission(reason, now)
            return False, reason
        approved = list(report["pose"])
        self._transition("park", "mission_accepted", now, preserve_mission=True)
        self._auto_reboot = health["m4"]["reboot_count"]
        self._mission_status.update(state="parking", reason="mission_accepted",
                                    pose=approved[:], approved_pose=approved[:], offer_id=None)
        return True, "mission_accepted"

    def _auto_health_reason(self, health):
        reboot = health.get("m4", {}).get("reboot_count")
        if not _integer(reboot) or self._auto_reboot is None or reboot != self._auto_reboot:
            return "auto_park_m4_reboot"
        for direction in DIRECTIONS:
            sensor = health.get("tof", {}).get(direction, {})
            if sensor.get("valid") is not True or not _finite(sensor.get("distance_mm"), 0, 65535):
                return "auto_park_tof_invalid_" + direction
            if sensor["distance_mm"] <= max(450, self._auto_config["tof_clearance_mm"]):
                return "auto_park_path_blocked"
        return None

    def _auto_expiry_reason(self, proposal, now):
        if now - self._entered > self._auto_config["mission_timeout_s"]:
            return "auto_park_timeout"
        if proposal is None:
            return "auto_park_worker_timeout" if now - self._entered > AUTO_WAIT_TIMEOUT else None
        state = proposal["state"]
        moving = proposal["esc"] != NEUTRAL[0]
        if (state in ("searching", "acquiring")
                and now - self._entered > self._auto_config["search_timeout_s"]):
            return "auto_park_search_timeout"
        if state == "planning":
            if (self._auto_plan_started is not None
                    and now - self._auto_plan_started > self._auto_config["planning_timeout_s"]):
                return "auto_park_plan_timeout"
            # 沒有移動的規劃計算可暫用同一份資料，期限不可隨新報告延長。
            if self._auto_moved:
                return "auto_park_state_invalid"
            return None
        if moving or self._auto_moved or proposal.get("completed"):
            ttl = min(AUTO_SOURCE_TTL, self._auto_config["proposal_timeout_s"])
            if now - proposal["source_at"] > ttl or now - proposal["input_at"] > ttl:
                return "auto_park_source_stale"
        elif now - proposal["input_at"] > AUTO_WAIT_TIMEOUT:
            return "auto_park_worker_timeout"
        return None

    def update_parking(self, report, now=None):
        """只接收本模式代次的新提案，不做規劃、不寫 UART、不接受過期續命。"""
        with self._lock:
            now = self._now(now)  # 取得鎖後才判斷有效期。
            if (not self._auto_enabled or self._mode != "park" or not isinstance(report, dict)
                    or report.get("session") != self.session or report.get("epoch") != self._epoch):
                return False
            expired = self._auto_expiry_reason(self._auto_proposal, now)
            if expired:
                self._abort_auto(expired, now)
                return False
            times = [report.get(key) for key in ("source_at", "input_at", "created_at")]
            esc, servo = report.get("esc"), report.get("servo")
            config = self._auto_config
            servo_min = min(config["steering_left_pwm"], config["steering_right_pwm"], 152)
            servo_max = max(config["steering_left_pwm"], config["steering_right_pwm"], 152)
            valid = (all(_finite(value) for value in times) and times[0] <= times[1] <= times[2] <= now
                     and times[1] >= self._entered
                     and _integer(report.get("epoch"))
                     and _integer(esc, ESC_MIN, ESC_MAX)
                     and esc in (152, config["forward_esc"], config["reverse_esc"])
                     and _integer(servo, servo_min, servo_max)
                     and type(report.get("stop")) is bool and type(report.get("completed")) is bool
                     and report.get("state") in AUTO_STATES
                     and isinstance(report.get("reason"), str) and 0 < len(report["reason"]) <= 80
                     and report.get("range_source") == "lidar_tof")
            if valid and report["stop"]:
                valid = (esc, servo) == NEUTRAL
            if valid and report["completed"]:
                valid = report["stop"] and report["state"] == "complete"
            if valid and report["state"] == "complete":
                valid = report["completed"] and report["stop"]
            if valid and report["state"] == "aborted":
                valid = report["stop"] and not report["completed"]
            if valid and report["state"] in ("searching", "acquiring", "planning", "gear_wait", "settling"):
                valid = (esc, servo) == NEUTRAL and not report["stop"]
            if valid and esc != 152:
                valid = report["state"] in ("aligning", "reversing") and not report["stop"]
            for key in ("pose", "target"):
                value = report.get(key)
                if value is not None and (not isinstance(value, (list, tuple)) or len(value) != 3
                                          or not all(_finite(v, -10000, 10000) for v in value)):
                    valid = False
            for key in ("progress_m", "path_remaining_m", "fit_error_m"):
                if key in report and not _finite(report[key], 0, 10000):
                    valid = False
            if not valid:
                self._abort_auto("auto_park_report_invalid", now)
                return False
            keys = ("source_at", "input_at", "created_at", "esc", "servo", "stop", "state",
                    "reason", "completed", "pose", "target", "progress_m", "path_remaining_m", "fit_error_m")
            if report["stop"] and not report["completed"]:
                # 同一份雷達幀仍可能使碰撞檢查判定中止；中止永遠優先於去重。
                decision = {key: copy.deepcopy(report[key]) for key in keys if key in report}
                self._transition("standby", report["reason"], now)
                self._parking_status = decision
                return True
            previous = self._auto_proposal
            # 相同或倒退的輸入不更新 last-seen；不能把舊幾何重新包裝成新油門。
            if previous is not None and report["input_at"] <= previous["input_at"]:
                return False
            if self._auto_source_high is not None and report["source_at"] <= self._auto_source_high:
                if report["esc"] != 152 or report["state"] != "planning":
                    return False
            if report["state"] == "planning" and self._auto_plan_started is None:
                self._auto_plan_started = now
            reason = self._auto_expiry_reason(report, now)
            if reason:
                self._abort_auto(reason, now)
                return False
            self._auto_proposal = {key: copy.deepcopy(report[key]) for key in keys if key in report}
            self._auto_source_high = max(report["source_at"], self._auto_source_high or 0)
            self._parking_status = dict(self._auto_proposal)
            if report["stop"]:
                decision = dict(self._parking_status)
                mission_complete = self._mission_status["active"] and report["completed"]
                self._transition("standby", report["reason"], now)
                self._parking_status = decision
                if mission_complete:
                    self._mission_status.update(state="completed", reason=report["reason"])
            return True

    def _auto_output(self, health, now):
        reason = self._auto_health_reason(health) or self._auto_expiry_reason(self._auto_proposal, now)
        if reason:
            self._abort_auto(reason, now)
            return NEUTRAL
        if self._auto_proposal is None:
            return NEUTRAL
        decision = self._auto_proposal
        values = (decision["esc"], decision["servo"])
        if values[0] != 152:
            self._auto_moved = True
        return values

    def _advance(self, health, now):
        self._observe_perception(health)
        m4 = health.get("m4", {}) if isinstance(health, dict) else {}
        self._mission_m4_neutral = (m4.get("connected") is True
                                    and m4.get("output_esc") == 152
                                    and m4.get("output_servo") == 152)
        self._mission_health_at = now
        if self._mode == "offer" and self._mission_m4_neutral:
            if self._mission_neutral_since is None:
                self._mission_neutral_since = now
        else:
            self._mission_neutral_since = None
        if self._mode not in ACTIVE_MODES:
            return
        reason = self._readiness_reason(health, now)
        if reason is None and self._mode in ("patrol", "search", "offer"):
            reason = self._patrol_camera_reason(now)
        if reason is None and self._mode == "park":
            reason = self._parking_camera_reason(now)
        if reason:
            self._transition("fault", reason, now)
            return
        if self._mode == "park" and self._auto_enabled:
            reason = self._auto_health_reason(health) or self._auto_expiry_reason(self._auto_proposal, now)
            if reason:
                self._abort_auto(reason, now)
                return
        if self._mode in ("search", "offer"):
            reason = self._mission_health_reason(health) or self._mission_expiry_reason(now)
            if reason:
                self._abort_mission(reason, now)
                return
            if (self._mode == "offer" and self._mission_status["state"] == "awaiting"
                    and not self._mission_m4_neutral):
                self._abort_mission("mission_not_stationary", now)
                return
        previous = {"manual": self._last_drive, "patrol": self._last_patrol,
                    "park": self._last_park, "search": self._last_mission,
                    "offer": self._last_mission}[self._mode]
        previous = self._entered if previous is None else previous
        if previous is None or now - previous > self.timeout:
            self._transition("fault", self._mode + "_input_timeout", now)

    def fault(self, reason, now=None):
        now = self._now(now)
        with self._lock:
            if self._mode != "fault" or self._reason != str(reason):
                self._transition("fault", str(reason)[:100], now)

    @staticmethod
    def _valid_packet(packet):
        if not isinstance(packet, dict) or packet.get("v") != 1 or isinstance(packet.get("v"), bool):
            return False
        if packet.get("type") != "control":
            return False
        if not _integer(packet.get("seq")) or not _integer(packet.get("epoch")):
            return False
        if not _integer(packet.get("esc"), ESC_MIN, ESC_MAX):
            return False
        if not _integer(packet.get("servo"), SERVO_MIN, SERVO_MAX):
            return False
        action = packet.get("action")
        if action is not None:
            if (not isinstance(action, dict) or not _integer(action.get("id"), 1)
                    or not _integer(action.get("epoch")) or not isinstance(action.get("name"), str)
                    or len(action["name"]) > 32):
                return False
        return True

    def _perform_action(self, action, packet, health, now):
        action_id, name = action["id"], action["name"]
        signature = (name, action["epoch"], action.get("offer_id"))
        if action_id in self._action_cache:
            previous_signature, previous_result = self._action_cache[action_id]
            if signature == previous_signature:
                self._last_action = copy.deepcopy(previous_result)
            else:
                self._last_action = dict(id=action_id, name=name, accepted=False, reason="action_id_conflict")
            return
        accepted, reason = False, "unknown_action"
        if action_id <= self._action_high:
            reason = "stale_action"
        elif name == "stop":
            # 停止不受過期模式版本阻擋，但仍須符合本次 session 與新封包序號。
            if self._mode == "fault":
                self._transition("fault", self._reason, now)
            else:
                self._transition("standby", "operator_stop", now)
            accepted, reason = True, "stopped"
        elif action["epoch"] != self._epoch or packet["epoch"] != self._epoch:
            reason = "epoch_mismatch"
        elif name in ("park_accept", "park_decline"):
            accepted, reason = self._decide_mission(action, packet, health, now)
        elif name in ("manual", "patrol", "park", "search"):
            if self._mode == "fault":
                reason = "reset_required"
            elif ((name == "park" and self._auto_present and not self._auto_enabled)
                  or (name == "search" and not self._auto_enabled)):
                reason = "auto_park_not_configured"
            elif name == "search" and self._mode != "standby":
                reason = "mission_requires_standby"
            elif ((name == "park" and self._mode != "standby")
                  or (name == "patrol" and self._mode in ("park", "search", "offer"))):
                reason = "park_requires_standby"
            elif (packet["esc"], packet["servo"]) != NEUTRAL:
                reason = "neutral_required"
            else:
                reason = self._readiness_reason(health, now)
                if reason is None and name in ("patrol", "search"):
                    reason = self._patrol_camera_reason(now)
                if reason is None and name == "park":
                    reason = self._parking_camera_reason(now)
                if reason is None and name == "search":
                    reason = self._mission_health_reason(health, starting=True)
                if reason is None:
                    self._transition(name, name + "_selected", now)
                    if name == "search":
                        self._start_mission(health, now)
                    if name == "park" and self._auto_enabled:
                        self._auto_reboot = health.get("m4", {}).get("reboot_count")
                    accepted, reason = True, name + "_selected"
        elif name == "reset":
            if self._mode in ACTIVE_MODES:
                reason = "stop_before_reset"
            elif (packet["esc"], packet["servo"]) != NEUTRAL:
                reason = "neutral_required"
            else:
                reason = self._readiness_reason(health, now)
                if reason is None:
                    self._transition("standby", "fault_reset", now)
                    accepted, reason = True, "fault_reset"
        self._last_action = dict(id=action_id, name=name, accepted=accepted, reason=reason)
        self._action_high = max(self._action_high, action_id)
        self._action_cache[action_id] = (signature, copy.deepcopy(self._last_action))
        while len(self._action_cache) > 32:
            self._action_cache.popitem(last=False)

    def receive(self, packet, health, now=None):
        with self._lock:
            # 確認期限在取得控制鎖後判斷；等待 UART 寫入不能保留過期的 Y。
            now = self._now(now)
            self._advance(health, now)
            if (not self._valid_packet(packet) or packet.get("session") != self.session
                    or packet["seq"] <= self._last_seq):
                self._rejected += 1
                return False
            self._last_seq, self._last_rx = packet["seq"], now
            action = packet.get("action")
            previous_epoch = self._epoch
            if action is not None:
                self._perform_action(action, packet, health, now)
            if self._epoch != previous_epoch:
                return True  # 切換本身不能同時成為新模式的第一筆行駛命令。
            if packet["epoch"] != self._epoch:
                return True
            if self._mode in ("patrol", "park", "search", "offer"):
                # 自主模式只用本代次的新中立封包續命，舊手動油門不能維持行駛。
                if (packet["esc"], packet["servo"]) == NEUTRAL:
                    if self._mode == "patrol":
                        self._last_patrol = now
                    elif self._mode == "park":
                        self._last_park = now
                    else:
                        self._last_mission = now
                    self._neutral_seen = True
                return True
            if self._mode != "manual":
                return True
            self._last_drive = now
            values = (packet["esc"], packet["servo"])
            if values == NEUTRAL:
                self._neutral_seen = True
            if not self._neutral_seen or now - self._entered < self.handoff:
                self._drive = NEUTRAL
            else:
                self._drive = values
            return True

    def output(self, health, now=None):
        with self._lock:
            now = self._now(now)
            self._advance(health, now)
            if (self._mode == "manual" and self._neutral_seen and self._last_drive is not None
                    and now - self._entered >= self.handoff):
                self._output = self._drive
            elif (self._mode in ("patrol", "search") and self._neutral_seen
                  and (self._last_patrol if self._mode == "patrol" else self._last_mission) is not None
                  and now - self._entered >= self.handoff):
                if self._mode == "search" and self._mission_report is None:
                    self._output = NEUTRAL
                    return self._output
                decision = self._planner.step(health, now=now)
                self._patrol_status = dict(decision)
                if decision["stop"]:
                    # 空間不足退出巡邏；距離恢復也不自動起步，需再次按 P。
                    self._transition("standby", decision["reason"], now)
                    self._patrol_status = dict(decision)
                else:
                    self._output = (decision["esc"], decision["servo"])
            elif (self._mode == "park" and self._neutral_seen and self._last_park is not None
                  and now - self._entered >= self.handoff):
                if self._auto_enabled:
                    self._output = self._auto_output(health, now)
                    return self._output
                decision = self._parking.step(health, now=now)
                self._parking_status = dict(decision)
                if decision["stop"]:
                    self._transition("standby", decision["reason"], now)
                    # 完成或中止資訊留給操作者查看，距離恢復也不重新倒車。
                    self._parking_status = dict(decision)
                else:
                    self._output = (decision["esc"], decision["servo"])
            else:
                self._output = NEUTRAL
            return self._output

    def dispatch_output(self, read_health, write_output, now=None):
        """唯一 UART 寫入者使用：重新判斷至完成送出間不允許切換模式。

        read_health 只可取得感測狀態，不可進行裝置 I/O；write_output
        必須採短逾時，且不可回呼控制狀態異動。鎖順序為控制→感測；
        其他路徑不可持感測鎖再取得控制鎖。
        故障先提交便輸出中立；若傳輸先開始，故障需等待這次寫入結束。
        本函式不保證取消已交給作業系統或已在導線上的封包。
        """
        with self._lock:
            health = read_health()
            # 不能用進入函式前的時間，否則等鎖後舊命令仍可能被當成有效。
            stamp = self._now(now)
            values = self.output(health, now=stamp)
            try:
                write_output(*values)
            except Exception:
                self._transition("fault", "m4_transport_error", self._now(now))
                raise
            return values

    def snapshot(self, now=None):
        now = self._now(now)
        with self._lock:
            input_age = None if self._last_drive is None else max(0, now - self._last_drive) * 1000
            remote_age = None if self._last_rx is None else max(0, now - self._last_rx) * 1000
            ready = (self._mode == "manual" and self._neutral_seen and self._last_drive is not None
                     and now - self._entered >= self.handoff and input_age <= self.timeout * 1000
                     and self._video_reason(now) is None)
            patrol = dict(self._patrol_status)
            patrol.update(active=self._mode == "patrol", range_source="tof",
                          ready=(self._mode == "patrol" and self._neutral_seen
                                 and self._last_patrol is not None
                                 and now - self._entered >= self.handoff
                                 and now - self._last_patrol <= self.timeout
                                 and self._video_reason(now) is None
                                 and self._patrol_camera_reason(now) is None),
                          heartbeat_age_ms=(None if self._last_patrol is None
                                            else max(0, now-self._last_patrol)*1000))
            parking = dict(self._parking_status)
            if self._auto_present and not self._auto_enabled:
                parking["reason"] = "auto_park_not_configured"
            parking.update(active=self._mode == "park", range_source="lidar_tof" if self._auto_present else "rear_tof",
                           ready=(self._mode == "park" and self._neutral_seen
                                  and self._last_park is not None
                                  and now - self._entered >= self.handoff
                                  and now - self._last_park <= self.timeout
                                  and self._video_reason(now) is None
                                  and self._parking_camera_reason(now) is None),
                           heartbeat_age_ms=(None if self._last_park is None
                                             else max(0, now-self._last_park)*1000))
            return {
                "mode": self._mode, "reason": self._reason, "epoch": self._epoch,
                "last_seq": self._last_seq, "last_action": copy.deepcopy(self._last_action),
                "manual_ready": ready, "input_age_ms": input_age, "remote_age_ms": remote_age,
                "output_esc": self._output[0], "output_servo": self._output[1],
                "capabilities": {"manual": True, "patrol": True,
                                 "park": not self._auto_present or self._auto_enabled,
                                 "search": self._auto_enabled},
                "perception": self._perception_snapshot(now), "patrol": patrol, "parking": parking,
                "mission": self._mission_snapshot(now),
                "rejected_packets": self._rejected, "transition_count": self._transitions,
                "last_transition": copy.deepcopy(self._last_transition),
                "video": self._video_snapshot(now),
            }
