#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""智慧車展示紀錄：唯讀取樣快照，與控制、UART、推論程序完全分離。"""
import argparse
import csv
import datetime
import fcntl
import io
import json
import math
import os
from pathlib import Path
import signal
import stat
import time
import zipfile


PRODUCER = "MiniTesla.demo_recorder"
PREFIX = "minitesla-demo"
MAX_FILE_BYTES = 10 * 1024 * 1024
MAX_BACKUPS = 4
MAX_SNAPSHOT_BYTES = 8 * 1024 * 1024
DIRECTIONS = ("front", "rear", "left", "right")
DEFAULT_LIMITS = {"tof": 120.0, "m4": 250.0, "lidar": 500.0, "camera": 1000.0}
ROOT = Path(__file__).resolve().parent


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="milliseconds")


def boot_id():
    try:
        return Path("/proc/sys/kernel/random/boot_id").read_text(encoding="ascii").strip()
    except OSError:
        return None


def mapping(value):
    return value if isinstance(value, dict) else {}


def number(value):
    try:
        return (isinstance(value, (int, float)) and not isinstance(value, bool)
                and math.isfinite(value))
    except OverflowError:
        return False


def scalar(value):
    if value is None or isinstance(value, (bool, int)):
        return value
    if isinstance(value, str):
        return value[:512]
    if isinstance(value, float) and math.isfinite(value):
        return value
    return None


def select(source, names):
    source = mapping(source)
    return {name: scalar(source.get(name)) for name in names}


def pose3(value):
    """僅保存有限的三維位姿摘要，不記錄點雲或控制物件。"""
    if (not isinstance(value, (list, tuple)) or len(value) != 3
            or not all(number(v) and abs(v) <= 10000 for v in value)):
        return None
    return [float(v) for v in value]


def reject_constant(value):
    raise ValueError("非有限數值：" + value)


def json_line(value):
    return (json.dumps(value, ensure_ascii=False, allow_nan=False,
                       separators=(",", ":")) + "\n").encode("utf-8")


def write_all(stream, data):
    remaining = memoryview(data)
    while remaining:
        count = stream.write(remaining)
        if count is None or count <= 0:
            raise OSError("紀錄寫入未完成")
        remaining = remaining[count:]


def read_snapshot(path):
    try:
        with Path(path).open("rb") as stream:
            raw = stream.read(MAX_SNAPSHOT_BYTES + 1)
        if len(raw) > MAX_SNAPSHOT_BYTES:
            return None, "snapshot_too_large"
        value = json.loads(raw.decode("utf-8"), parse_constant=reject_constant)
        return (value, None) if isinstance(value, dict) else (None, "snapshot_invalid")
    except FileNotFoundError:
        return None, "snapshot_missing"
    except (OSError, ValueError, UnicodeError, RecursionError):
        return None, "snapshot_unreadable"


def sensor_health(source, age_increment_ms, limit, validity="valid"):
    source = mapping(source)
    age = source.get("age_ms")
    age = float(age) if number(age) and age >= 0 else None
    adjusted = None if age is None else age + age_increment_ms
    valid_at_snapshot = source.get(validity) is True
    valid = valid_at_snapshot and adjusted is not None and adjusted <= limit
    return {"valid_at_snapshot": valid_at_snapshot, "valid": valid,
            "age_ms_at_snapshot": age, "age_ms": adjusted}


def compact_snapshot(source, age_ms):
    """目前健康狀態與快照當下健康狀態分開，避免把取樣延遲誤認為控制故障。"""
    control = mapping(source.get("control"))
    limits = dict(DEFAULT_LIMITS)
    for name, value in mapping(source.get("recording_thresholds_ms")).items():
        if name in limits and number(value) and 0 < value <= 60000:
            limits[name] = float(value)
    m4_source = mapping(source.get("m4"))
    m4 = sensor_health(m4_source, age_ms, limits["m4"], "connected")
    m4.update(select(m4_source, ("mode", "mode_name", "reasons_mask", "reboot_count",
                                  "output_esc", "output_servo", "mcu_uptime_ms")))
    m4["state_current"] = m4["valid"] and m4_source.get("state_current") is True
    reasons = m4_source.get("reasons")
    m4["reasons_at_snapshot"] = [str(v)[:128] for v in reasons[:32]] if isinstance(reasons, list) else []
    tof = {}
    for direction in DIRECTIONS:
        item = mapping(mapping(source.get("tof")).get(direction))
        health = sensor_health(item, age_ms, limits["tof"])
        health["valid"] = health["valid"] and m4["valid"]
        health.update(select(item, ("reason", "sample_uptime_ms", "guard_obstacle")))
        distance = item.get("distance_mm")
        health["distance_mm_at_snapshot"] = distance if number(distance) and distance >= 0 else None
        health["valid"] = health["valid"] and health["distance_mm_at_snapshot"] is not None
        health["distance_mm"] = health["distance_mm_at_snapshot"] if health["valid"] else None
        tof[direction] = health
    lidar = sensor_health(source.get("lidar"), age_ms, limits["lidar"])
    lidar.update(select(source.get("lidar"), ("orientation_configured", "packets", "invalid_returns")))
    camera = sensor_health(source.get("camera"), age_ms, limits["camera"])
    perception_source = mapping(source.get("perception"))
    perception = sensor_health(perception_source, age_ms, limits["camera"])
    perception.update(select(perception_source, ("event_count", "last_event_at")))
    perception["person_count_at_snapshot"] = scalar(perception_source.get("person_count"))
    perception["person_count"] = perception["person_count_at_snapshot"] if perception["valid"] else None
    perception["person_present"] = scalar(perception_source.get("person_present")) if perception["valid"] else None
    compact_control = select(control, ("mode", "reason", "epoch", "output_esc", "output_servo",
                                       "manual_ready", "transition_count", "rejected_packets"))
    compact_control["patrol"] = select(control.get("patrol"), (
        "state", "reason", "active", "ready", "esc", "servo", "stop", "range_source"))
    compact_control["parking"] = select(control.get("parking"), (
        "state", "reason", "active", "ready", "esc", "servo", "stop", "completed",
        "rear_mm", "target_mm", "progress_mm", "range_source", "progress_m",
        "path_remaining_m", "fit_error_m"))
    for name in ("pose", "target"):
        compact_control["parking"][name] = pose3(mapping(control.get("parking")).get(name))
    compact_control["mission"] = select(control.get("mission"), (
        "active", "state", "reason", "offer_id", "remaining_s", "decision_s", "fit_error_m"))
    compact_control["mission"]["pose"] = pose3(mapping(control.get("mission")).get("pose"))
    compact_control["video"] = select(control.get("video"), (
        "state", "ready", "reason", "frame_age_ms", "ack_age_ms", "frame_seq", "disconnect_count", "timeout_ms"))
    video = compact_control["video"]
    video["ready_at_snapshot"] = video["ready"] is True
    for age_name in ("frame_age_ms", "ack_age_ms"):
        value = video[age_name]
        video[age_name + "_at_snapshot"] = value
        video[age_name] = value + age_ms if number(value) and value >= 0 else None
    video["ready"] = (video["ready_at_snapshot"] and number(video["timeout_ms"])
                      and all(number(video[name]) and video[name] <= video["timeout_ms"]
                              for name in ("frame_age_ms", "ack_age_ms")))
    runtime = select(source.get("runtime"), ("state", "reason"))
    operating = runtime["state"] == "running"
    if not operating:
        for health in [m4, lidar, camera, perception] + list(tof.values()):
            health["valid"] = False
        m4["state_current"] = False
        video["ready"] = False
        perception["person_count"] = perception["person_present"] = None
        for health in tof.values():
            health["distance_mm"] = None
    errors = {str(k)[:64]: select(v, ("message", "monotonic_s"))
              for k, v in list(mapping(source.get("errors")).items())[:16]}
    return {"runtime": runtime, "operating": operating,
            "control": compact_control, "m4": m4, "tof": tof, "lidar": lidar,
            "camera": camera, "perception": perception, "errors": errors,
            "recording_thresholds_ms": limits}


class SnapshotSampler:
    """單調時鐘 + 開機識別驗證；僅對新快照取樣，不重複累計人物事件。"""
    def __init__(self, current_boot_id=None, stale_after=1.0):
        self.boot = boot_id() if current_boot_id is None else current_boot_id
        self.stale_after = stale_after
        self.last_key = None
        self.last_run = None
        self.last_status = None
        self.last_unavailable = None
        self.previous = {}

    def process(self, source, error=None, now=None, recorded_at=None):
        now = time.monotonic() if now is None else now
        recorded_at = utc_now() if recorded_at is None else recorded_at
        status = error
        source = mapping(source)
        stamp, run = source.get("monotonic_s"), source.get("run_id")
        age = None
        if status is None:
            if not self.boot or source.get("boot_id") != self.boot:
                status = "snapshot_boot_mismatch"
            elif not isinstance(run, str) or not run or len(run) > 128 or not number(stamp):
                status = "snapshot_invalid"
            elif not number(source.get("timestamp_unix")):
                status = "snapshot_invalid"
            else:
                age = now - stamp
                status = ("snapshot_clock_invalid" if age < 0 else
                          "snapshot_stale" if age >= self.stale_after else "fresh")
                if status == "fresh" and self.last_run == run and self.last_key is not None and stamp < self.last_key[1]:
                    status = "snapshot_time_regression"
        base = {"schema_version": 1, "producer": PRODUCER, "recorded_at": recorded_at,
                "recorder_monotonic_s": now, "source_status": status,
                "run_id": run if isinstance(run, str) and len(run) <= 128 else None}
        result = []
        if status != self.last_status:
            result.append(dict(base, kind="event", event="source_status",
                               previous=self.last_status, current=status))
            self.last_status = status
            self.last_unavailable = now
        if status != "fresh":
            self.previous = {}
            if self.last_unavailable is None or now - self.last_unavailable >= 1.0:
                result.append(dict(base, kind="source_unavailable", snapshot_age_ms=(None if age is None else age * 1000)))
                self.last_unavailable = now
            return result
        key = (run, stamp)
        if key == self.last_key:
            return result
        if run != self.last_run:
            result.append(dict(base, kind="event", event="source_run", previous=self.last_run, current=run))
            self.previous = {}
            self.last_run = run
        self.last_key = key
        compact = compact_snapshot(source, age * 1000)
        sample = dict(base, kind="sample", snapshot_monotonic_s=stamp,
                      snapshot_timestamp_unix=source["timestamp_unix"], snapshot_age_ms=age * 1000, **compact)
        result.append(sample)
        control = compact["control"]
        transitions = {
            "runtime": compact["runtime"],
            "control": select(control, ("mode", "reason", "epoch", "transition_count")),
            "m4_guard": {"valid_at_snapshot": compact["m4"]["valid_at_snapshot"],
                         "mode_name": compact["m4"]["mode_name"],
                         "reasons_mask": compact["m4"]["reasons_mask"],
                         "reboot_count": compact["m4"]["reboot_count"]},
            "person_alert": select(compact["perception"], ("valid_at_snapshot", "person_count_at_snapshot", "event_count")),
            "patrol": select(control["patrol"], ("state", "reason", "active")),
            "parking": select(control["parking"], ("state", "reason", "active", "completed")),
            "mission": select(control["mission"], ("state", "reason", "active", "offer_id")),
        }
        for name, value in transitions.items():
            if value != self.previous.get(name):
                result.append(dict(base, kind="event", event=name,
                                   previous=self.previous.get(name), current=value))
        self.previous = transitions
        return result


def record_path(directory, index=0):
    name = PREFIX + (".{}".format(index) if index else "") + ".jsonl"
    return Path(directory) / name


def open_regular(path, flags, mode=0o600):
    descriptor = os.open(str(path), flags | getattr(os, "O_NOFOLLOW", 0), mode)
    if not stat.S_ISREG(os.fstat(descriptor).st_mode):
        os.close(descriptor)
        raise ValueError("不是一般檔案：{}".format(path))
    return descriptor


def validate_owned(stream, path):
    stream.seek(0)
    first = stream.readline(4096)
    try:
        header = json.loads(first.decode("utf-8"))
    except (ValueError, UnicodeError):
        raise ValueError("紀錄名稱已被其他檔案使用：{}".format(path))
    if not isinstance(header, dict) or header.get("producer") != PRODUCER or header.get("kind") != "file_header":
        raise ValueError("拒絕覆蓋非智慧車紀錄：{}".format(path))


class BoundedWriter:
    def __init__(self, directory, max_bytes=MAX_FILE_BYTES, backups=MAX_BACKUPS):
        if not 1024 <= max_bytes <= MAX_FILE_BYTES or not 0 <= backups <= MAX_BACKUPS:
            raise ValueError("每檔上限需為 1024..10485760 位元組，保留份數需為 0..4")
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.max_bytes, self.backups = max_bytes, backups
        self.stream = None
        self.lock = self.rotation_lock = None
        try:
            self.lock = open_regular(self.directory / ".minitesla-recorder.lock", os.O_CREAT | os.O_RDWR)
            try:
                fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise RuntimeError("此紀錄資料夾已有另一個 recorder 執行中")
            self.rotation_lock = open_regular(self.directory / ".minitesla-records.lock", os.O_CREAT | os.O_RDWR)
            fcntl.flock(self.rotation_lock, fcntl.LOCK_EX)
            try:
                for index in range(MAX_BACKUPS + 1):
                    path = record_path(self.directory, index)
                    if path.exists() or path.is_symlink():
                        with os.fdopen(open_regular(path, os.O_RDONLY), "rb") as existing:
                            validate_owned(existing, path)
                            if os.fstat(existing.fileno()).st_size > max_bytes:
                                raise ValueError("既有紀錄大於 --max-bytes；請保留原容量設定")
                        if index > backups:
                            path.unlink()
                self._open()
            finally:
                fcntl.flock(self.rotation_lock, fcntl.LOCK_UN)
        except BaseException:
            self.close()
            raise

    def _open(self):
        path = record_path(self.directory)
        descriptor = open_regular(path, os.O_CREAT | os.O_RDWR | os.O_APPEND)
        self.stream = os.fdopen(descriptor, "a+b", buffering=0)
        size = os.fstat(descriptor).st_size
        if size:
            # 中斷寫入留下的半行先移除；保留前面完整的 JSONL。
            self.stream.seek(max(0, size - self.max_bytes - 1))
            tail = self.stream.read()
            last_newline = tail.rfind(b"\n")
            keep = size - len(tail) + last_newline + 1
            if last_newline < 0:
                raise ValueError("紀錄缺少完整 JSONL 行")
            if keep < size:
                self.stream.truncate(keep)
        else:
            write_all(self.stream, json_line({"kind": "file_header", "producer": PRODUCER,
                                             "schema_version": 1, "created_at": utc_now()}))

    def _rotate(self):
        self.stream.close()
        self.stream = None
        oldest = record_path(self.directory, self.backups)
        if oldest.exists():
            oldest.unlink()
        for index in range(self.backups - 1, -1, -1):
            source = record_path(self.directory, index)
            if source.exists():
                os.replace(str(source), str(record_path(self.directory, index + 1)))
        self._open()

    def write(self, record):
        data = json_line(record)
        if len(data) + 256 > self.max_bytes:
            raise ValueError("單筆紀錄大於每檔容量；請提高 --max-bytes")
        fcntl.flock(self.rotation_lock, fcntl.LOCK_EX)
        try:
            if os.fstat(self.stream.fileno()).st_size + len(data) > self.max_bytes:
                self._rotate()
            write_all(self.stream, data)
        finally:
            fcntl.flock(self.rotation_lock, fcntl.LOCK_UN)

    def close(self):
        if self.stream is not None:
            self.stream.close()
            self.stream = None
        for name in ("rotation_lock", "lock"):
            descriptor = getattr(self, name, None)
            if descriptor is not None:
                os.close(descriptor)
                setattr(self, name, None)


SAMPLE_FIELDS = ["recorded_at", "run_id", "snapshot_monotonic_s", "snapshot_age_ms", "operating",
                 "runtime_state", "runtime_reason", "mode", "control_reason", "epoch",
                 "output_esc", "output_servo", "m4_valid", "m4_valid_at_snapshot",
                 "m4_age_ms", "m4_mode", "m4_reasons", "lidar_valid", "camera_valid",
                 "person_count", "person_count_at_snapshot", "person_event_count",
                 "video_ready", "video_frame_age_ms", "video_ack_age_ms", "video_disconnect_count",
                 "patrol_state", "parking_state", "parking_reason", "parking_completed"]
for _direction in DIRECTIONS:
    SAMPLE_FIELDS.extend([_direction + "_mm", _direction + "_mm_at_snapshot",
                          _direction + "_valid", _direction + "_valid_at_snapshot", _direction + "_age_ms"])
SAMPLE_FIELDS.extend(["mission_state", "mission_reason", "mission_offer_id", "mission_remaining_s",
                      "mission_pose_x_m", "mission_pose_y_m", "mission_pose_yaw_rad",
                      "parking_pose_x_m", "parking_pose_y_m", "parking_pose_yaw_rad",
                      "parking_target_x_m", "parking_target_y_m", "parking_target_yaw_rad",
                      "parking_progress_m", "parking_path_remaining_m", "parking_fit_error_m"])
EVENT_FIELDS = ["recorded_at", "run_id", "source_status", "kind", "event", "previous", "current"]


def csv_sample(record):
    runtime, control = mapping(record.get("runtime")), mapping(record.get("control"))
    m4, perception = mapping(record.get("m4")), mapping(record.get("perception"))
    parking = mapping(control.get("parking"))
    row = {key: record.get(key) for key in ("recorded_at", "run_id", "snapshot_monotonic_s", "snapshot_age_ms", "operating")}
    row.update(runtime_state=runtime.get("state"), runtime_reason=runtime.get("reason"),
               mode=control.get("mode"), control_reason=control.get("reason"), epoch=control.get("epoch"),
               output_esc=control.get("output_esc"), output_servo=control.get("output_servo"),
               m4_valid=m4.get("valid"), m4_valid_at_snapshot=m4.get("valid_at_snapshot"),
               m4_age_ms=m4.get("age_ms"), m4_mode=m4.get("mode_name"),
               m4_reasons=json.dumps(m4.get("reasons_at_snapshot"), ensure_ascii=False),
               lidar_valid=mapping(record.get("lidar")).get("valid"), camera_valid=mapping(record.get("camera")).get("valid"),
               person_count=perception.get("person_count"), person_count_at_snapshot=perception.get("person_count_at_snapshot"),
               person_event_count=perception.get("event_count"),
               video_ready=mapping(control.get("video")).get("ready"),
               video_frame_age_ms=mapping(control.get("video")).get("frame_age_ms"),
               video_ack_age_ms=mapping(control.get("video")).get("ack_age_ms"),
               video_disconnect_count=mapping(control.get("video")).get("disconnect_count"),
               patrol_state=mapping(control.get("patrol")).get("state"), parking_state=parking.get("state"),
               parking_reason=parking.get("reason"), parking_completed=parking.get("completed"))
    mission = mapping(control.get("mission"))
    row.update(mission_state=mission.get("state"), mission_reason=mission.get("reason"),
               mission_offer_id=mission.get("offer_id"), mission_remaining_s=mission.get("remaining_s"),
               parking_progress_m=parking.get("progress_m"),
               parking_path_remaining_m=parking.get("path_remaining_m"),
               parking_fit_error_m=parking.get("fit_error_m"))
    for prefix, value in (("mission_pose", mission.get("pose")), ("parking_pose", parking.get("pose")),
                          ("parking_target", parking.get("target"))):
        vector = pose3(value)
        for i, suffix in enumerate(("x_m", "y_m", "yaw_rad")):
            row[prefix + "_" + suffix] = None if vector is None else vector[i]
    for direction in DIRECTIONS:
        item = mapping(mapping(record.get("tof")).get(direction))
        for target, source in (("mm", "distance_mm"), ("mm_at_snapshot", "distance_mm_at_snapshot"),
                               ("valid", "valid"), ("valid_at_snapshot", "valid_at_snapshot"), ("age_ms", "age_ms")):
            row[direction + "_" + target] = item.get(source)
    return row


EXPORT_README = """智慧車展示紀錄\n\n這是預設每 0.2 秒讀取快照的取樣紀錄，不是每個控制封包或全部狀態轉換的完整追蹤。\nrecords.jsonl 保留匯出時仍在輪替範圍內的完整 JSON 行；samples.csv 是感測與控制取樣，events.csv 是觀察到的變化。\n舊紀錄會依容量輪替刪除：預設每檔 10 MiB，含目前檔共 5 檔，最多 50 MiB。\nvalid_at_snapshot 是車端快照當下狀態；valid/age_ms 另加紀錄程序讀取延遲。紀錄程序不會影響車端控制判定。\n過期、不同開機識別或無法讀取的快照只寫狀態事件，不沿用先前健康值。\nperson_event_count 是每次車端程序內人物連續出現的警示次數，不是不同人物人數；重啟會重新計數，請連同 run_id 判讀。\n人物距離沒有由影像猜測。停車完成依車端判定，不代表已量得零車速。\n第十階段新增任務狀態、通知編號、倒數秒數及停車位姿摘要；位姿長度為公尺、朝向為弧度，原點在車位後壁中央。\n倒數與位姿為取樣當下值，不是離線匯出時的即時狀態。未保存完整規劃路徑，舊紀錄的新欄位留空。\n匯出不含影像、模型、完整偵測框或雷達點雲；不代表新增功能已經實車驗證。\n匯出時間點以鎖定後開啟的各檔案長度為準；後續新增行不在此次匯出內，尾端不完整行會略過。\n"""


def export_records(directory, destination=None):
    directory = Path(directory).expanduser().resolve()
    if not directory.is_dir():
        raise ValueError("找不到紀錄資料夾：{}".format(directory))
    handles = []
    rotation_lock = open_regular(directory / ".minitesla-records.lock", os.O_CREAT | os.O_RDWR)
    try:
        fcntl.flock(rotation_lock, fcntl.LOCK_SH)
        for index in range(MAX_BACKUPS, -1, -1):
            path = record_path(directory, index)
            try:
                stream = os.fdopen(open_regular(path, os.O_RDONLY), "rb")
            except FileNotFoundError:
                continue
            try:
                validate_owned(stream, path)
                size = os.fstat(stream.fileno()).st_size
                if size > MAX_FILE_BYTES:
                    raise ValueError("紀錄超過支援的 10 MiB 每檔上限")
                stream.seek(0)
                handles.append((stream, size, path.name))
            except BaseException:
                stream.close()
                raise
    except BaseException:
        for stream, _, _ in handles:
            stream.close()
        raise
    finally:
        fcntl.flock(rotation_lock, fcntl.LOCK_UN)
        os.close(rotation_lock)
    if not handles:
        raise ValueError("尚無智慧車展示紀錄可以匯出")
    try:
        if destination is None:
            stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
            destination = directory / ("MiniTesla_Demo_" + stamp + ".zip")
        destination = Path(destination).expanduser().resolve()
        if destination.suffix.lower() != ".zip":
            raise ValueError("匯出檔名必須以 .zip 結尾")
        destination.parent.mkdir(parents=True, exist_ok=True)
    except BaseException:
        for stream, _, _ in handles:
            stream.close()
        raise
    metadata = {"producer": PRODUCER, "schema_version": 1, "exported_at": utc_now(),
                "source_files": [{"name": name, "captured_bytes": size} for _, size, name in handles],
                "samples": 0, "events": 0, "skipped_lines": 0, "run_ids": [],
                "sampled_transitions_only": True, "recording_storage_limit_bytes": MAX_FILE_BYTES * (MAX_BACKUPS + 1)}
    seen_runs = set()
    try:
        # x 模式不覆蓋任何既有檔案；UTF-8 BOM 讓試算表直接辨識中文。
        with zipfile.ZipFile(str(destination), "x", compression=zipfile.ZIP_DEFLATED) as archive:
            samples = io.StringIO(newline="")
            events = io.StringIO(newline="")
            sample_writer, event_writer = csv.DictWriter(samples, SAMPLE_FIELDS), csv.DictWriter(events, EVENT_FIELDS)
            sample_writer.writeheader()
            event_writer.writeheader()
            with archive.open("records.jsonl", "w") as records:
                for stream, size, name in handles:
                    raw = stream.read(size)
                    complete = raw[:raw.rfind(b"\n") + 1]
                    if len(complete) != len(raw):
                        metadata["skipped_lines"] += 1
                    for line in complete.splitlines():
                        try:
                            record = json.loads(line.decode("utf-8"), parse_constant=reject_constant)
                            if not isinstance(record, dict) or record.get("producer") != PRODUCER:
                                raise ValueError("unknown record")
                        except (ValueError, UnicodeError, RecursionError):
                            metadata["skipped_lines"] += 1
                            continue
                        records.write(json_line(record))
                        run = record.get("run_id")
                        if isinstance(run, str):
                            seen_runs.add(run)
                        if record.get("kind") == "sample":
                            sample_writer.writerow(csv_sample(record))
                            metadata["samples"] += 1
                        elif record.get("kind") in ("event", "source_unavailable"):
                            row = {key: record.get(key) for key in EVENT_FIELDS}
                            for key in ("previous", "current"):
                                row[key] = json.dumps(row[key], ensure_ascii=False, allow_nan=False)
                            event_writer.writerow(row)
                            metadata["events"] += 1
            archive.writestr("samples.csv", samples.getvalue().encode("utf-8-sig"))
            archive.writestr("events.csv", events.getvalue().encode("utf-8-sig"))
            metadata["run_ids"] = sorted(seen_runs)
            archive.writestr("manifest.json", json.dumps(metadata, ensure_ascii=False, allow_nan=False, indent=2))
            archive.writestr("README.txt", EXPORT_README)
    finally:
        for stream, _, _ in handles:
            stream.close()
    return destination


def record(args):
    stop_requested = [False]
    def request_stop(_signum, _frame):
        # Python 訊號處理器不拿鎖；主迴圈自行結束並關閉檔案。
        stop_requested[0] = True
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, request_stop)
    writer = BoundedWriter(args.output_dir, args.max_bytes, args.backups)
    sampler = SnapshotSampler(stale_after=args.stale_after)
    print("[紀錄] {}；每 {:.2f} 秒取樣，不參與控制。".format(Path(args.output_dir).resolve(), args.interval), flush=True)
    try:
        while not stop_requested[0]:
            before = time.monotonic()
            source, error = read_snapshot(args.snapshot)
            for item in sampler.process(source, error):
                writer.write(item)
            deadline = before + args.interval
            while not stop_requested[0] and time.monotonic() < deadline:
                time.sleep(min(0.1, max(0, deadline - time.monotonic())))
    finally:
        writer.close()
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description="智慧車展示紀錄：獨立唯讀取樣與匯出")
    subparsers = parser.add_subparsers(dest="command", required=True)
    capture = subparsers.add_parser("record", help="持續取樣；由開機服務啟動")
    capture.add_argument("--snapshot", default=str(ROOT / "sensor_snapshot.json"))
    capture.add_argument("--output-dir", default=str(ROOT / "demo_records"))
    capture.add_argument("--interval", type=float, default=0.2)
    capture.add_argument("--stale-after", type=float, default=1.0)
    capture.add_argument("--max-bytes", type=int, default=MAX_FILE_BYTES)
    capture.add_argument("--backups", type=int, default=MAX_BACKUPS)
    export = subparsers.add_parser("export", help="匯出目前保留紀錄為 JSONL / CSV / 說明 ZIP")
    export.add_argument("--output-dir", default=str(ROOT / "demo_records"))
    export.add_argument("--destination")
    args = parser.parse_args(argv)
    try:
        if args.command == "record":
            if not number(args.interval) or not 0.05 <= args.interval <= 5:
                parser.error("--interval 必須為 0.05..5 秒")
            if not number(args.stale_after) or not 0.2 <= args.stale_after <= 5:
                parser.error("--stale-after 必須為 0.2..5 秒")
            return record(args)
        result = export_records(args.output_dir, args.destination)
        print("[匯出] {}\n紀錄為取樣結果，可能略過取樣間隔內的短暫狀態轉換。".format(result))
        return 0
    except (OSError, ValueError, RuntimeError) as exc:
        print("[紀錄錯誤] {}".format(exc), flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
