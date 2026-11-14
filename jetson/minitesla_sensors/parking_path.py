"""雷達可見自由空間內的有界 Hybrid A*；rear axle / x 前 / y 左 / 公尺。

curvature 是車輛向前行駛時的 d(yaw)/ds；倒車的 yaw 變化為
gear * curvature * 行駛距離，不可再把 pure pursuit 曲率乘一次 gear。
此模組只計算路徑與追蹤建議，不發送 UART / ESC 命令。
"""
import heapq
import math
import time

import numpy as np


TAU = 2.0 * math.pi


def wrap(angle):
    return (angle + math.pi) % TAU - math.pi


def finite_number(value):
    try:
        return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)
    except OverflowError:
        return False


def _settings(cfg):
    if not isinstance(cfg, dict):
        raise ValueError("path_config_invalid")
    for key in ("rear_overhang_m", "min_turn_radius_m", "bay_width_m", "bay_depth_m", "lidar_origin_body_m"):
        if key not in cfg:
            raise ValueError("path_config_missing:" + key)
    defaults = {"body_length_m": .474, "body_width_m": .328, "wheelbase_m": .275,
                "margin_m": .15, "grid_m": .08, "max_nodes": 20000,
                "time_budget_s": 3.0, "step_m": .16, "sweep_step_m": .04,
                "max_scan_gap_deg": 4.0, "max_range_m": 6.0,
                "goal_position_tolerance_m": .032, "goal_yaw_tolerance_deg": 2.4}
    result = dict(defaults)
    result.update(cfg)
    if "grid_resolution_m" in cfg:
        result["grid_m"] = cfg["grid_resolution_m"]
    if "scan_range_m" in cfg:
        result["max_range_m"] = cfg["scan_range_m"]
    # 規劃與追蹤各花最終容許誤差 40%，兩者相加仍保留 20% 餘量。
    for outer, inner in (("position_tolerance_m", "goal_position_tolerance_m"),
                         ("yaw_tolerance_deg", "goal_yaw_tolerance_deg")):
        if outer in cfg:
            if not finite_number(cfg[outer]):
                raise ValueError("path_config_invalid:" + outer)
            result[inner] = .4 * cfg[outer]
    for key in defaults:
        if not finite_number(result[key]):
            raise ValueError("path_config_invalid:" + key)
    for key in ("rear_overhang_m", "min_turn_radius_m", "bay_width_m", "bay_depth_m"):
        if not finite_number(result[key]):
            raise ValueError("path_config_invalid:" + key)
    bounds = {"body_length_m": (.2, 2.), "body_width_m": (.1, 1.), "wheelbase_m": (.1, 1.5),
              "margin_m": (.05, .5), "grid_m": (.04, .15), "time_budget_s": (.01, 3.),
              "step_m": (.08, .3), "sweep_step_m": (.01, .04), "max_scan_gap_deg": (.5, 4.),
              "max_range_m": (2., 8.), "bay_width_m": (.3, 5.), "bay_depth_m": (.4, 6.),
              "min_turn_radius_m": (.25, 4.), "goal_position_tolerance_m": (.015, .05),
              "goal_yaw_tolerance_deg": (.75, 3.2)}
    for key, (minimum, maximum) in bounds.items():
        if not minimum <= result[key] <= maximum:
            raise ValueError("path_config_out_of_range:" + key)
    if (not 0 < result["rear_overhang_m"] < result["body_length_m"] - result["wheelbase_m"]
            or result["body_width_m"] >= result["body_length_m"]):
        raise ValueError("path_config_invalid:vehicle_geometry")
    if isinstance(result["max_nodes"], bool) or int(result["max_nodes"]) != result["max_nodes"] or not 100 <= result["max_nodes"] <= 20000:
        raise ValueError("path_config_out_of_range:max_nodes")
    origin = np.asarray(result["lidar_origin_body_m"], dtype=float)
    if origin.shape != (2,) or not np.all(np.isfinite(origin)) or np.max(np.abs(origin)) > 2:
        raise ValueError("path_config_invalid:lidar_origin_body_m")
    result["lidar_origin_body_m"] = origin
    result["max_nodes"] = int(result["max_nodes"])
    return result


def _pose(value):
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.all(np.isfinite(result)) or np.max(np.abs(result[:2])) > 100:
        raise ValueError("path_pose_invalid")
    return (float(result[0]), float(result[1]), wrap(float(result[2])))


def advance(pose, distance, curvature):
    """精確定曲率 bicycle 弧線；distance 帶前進／倒車正負號。"""
    x, y, yaw = pose
    if abs(curvature) < 1e-12:
        return (x + distance * math.cos(yaw), y + distance * math.sin(yaw), yaw)
    next_yaw = yaw + distance * curvature
    return (x + (math.sin(next_yaw) - math.sin(yaw)) / curvature,
            y + (math.cos(yaw) - math.cos(next_yaw)) / curvature, wrap(next_yaw))


def interval_reduce(values, left, right, minimum=True):
    """向量化區間最小／最大值，區間為 [left,right)，不建立 rays × cells 陣列。"""
    combine = np.minimum if minimum else np.maximum
    lengths = right - left
    if np.any(lengths <= 0) or np.any(left < 0) or np.any(right > len(values)):
        raise ValueError("path_visibility_interval_invalid")
    levels = np.floor(np.log2(lengths)).astype(int)
    result = np.empty(left.shape, dtype=float)
    table = np.asarray(values, dtype=float)
    maximum = int(np.max(levels))
    for level in range(maximum + 1):
        mask = levels == level
        if np.any(mask):
            result[mask] = combine(table[left[mask]], table[right[mask] - (1 << level)])
        if level < maximum:
            width = 1 << level
            table = combine(table[:-width], table[width:])
    return result


def check_motion(pose, scan_points_body, cfg, gear, curvature, distance_m=.18):
    """以最新掃描驗證局部掃掠；distance_m 是前視路徑長度，不是煞停距離。"""
    began = time.monotonic()
    result = {"ok": False, "reason": "path_motion_invalid"}
    try:
        settings = _settings(cfg)
        current = _pose(pose)
        if isinstance(gear, bool) or gear not in (-1, 1) or not finite_number(curvature) or abs(curvature) > 1. / settings["min_turn_radius_m"] + 1e-9:
            raise ValueError("path_motion_invalid")
        if not finite_number(distance_m) or not .02 <= distance_m <= .5:
            raise ValueError("path_motion_distance_invalid")
        budget = settings.get("guard_time_budget_s", .25)
        if not finite_number(budget) or not .02 <= budget <= .5:
            raise ValueError("path_config_invalid:guard_time_budget_s")
        deadline = began + budget
        grid = VisibleGrid(current, scan_points_body, settings, deadline)
        clear = grid.swept(current, gear, curvature, distance_m)
        if time.monotonic() >= deadline:
            result["reason"] = "path_motion_timeout"
        else:
            result.update(ok=bool(clear), reason="path_motion_clear" if clear else "path_motion_blocked_or_unknown")
    except (ValueError, TypeError, OverflowError) as exc:
        result["reason"] = str(exc) if str(exc).startswith("path_") else "path_motion_invalid"
    result["elapsed_s"] = time.monotonic() - began
    return result


class VisibleGrid:
    """只有有效射線可證明自由空間；格子半對角線保護邊界與遮擋。"""
    def __init__(self, start, points, cfg, deadline):
        self.cfg, self.resolution = cfg, cfg["grid_m"]
        if not hasattr(points, "__len__") or not 32 <= len(points) <= 8192:
            raise ValueError("path_scan_invalid")
        raw = np.asarray(points, dtype=float)
        if raw.ndim != 2 or raw.shape[1:] != (2,) or not 32 <= len(raw) <= 8192:
            raise ValueError("path_scan_invalid")
        origin = cfg["lidar_origin_body_m"]
        rays = raw - origin
        finite = np.all(np.isfinite(rays), axis=1) & (np.max(np.abs(rays), axis=1) <= 1000)
        rays = rays[finite]
        lengths = np.linalg.norm(rays, axis=1)
        valid = lengths > .03
        rays, lengths = rays[valid], lengths[valid]
        if len(rays) < 32:
            raise ValueError("path_scan_insufficient")
        angles = np.mod(np.arctan2(rays[:, 1], rays[:, 0]) + start[2], TAU)
        order = np.argsort(angles)
        angles, lengths = angles[order], np.minimum(lengths[order], cfg["max_range_m"])
        unique, first = np.unique(angles, return_index=True)
        lengths = np.minimum.reduceat(lengths, first)
        angles = unique
        if len(angles) < 32:
            raise ValueError("path_scan_insufficient")
        co, si = math.cos(start[2]), math.sin(start[2])
        self.sensor = np.array([start[0] + co * origin[0] - si * origin[1],
                                start[1] + si * origin[0] + co * origin[1]])
        extent = cfg["max_range_m"] + self.resolution
        self.lower = np.floor((self.sensor - extent) / self.resolution) * self.resolution
        count = int(math.ceil(2 * extent / self.resolution)) + 1
        if count * count > 180000:
            raise ValueError("path_grid_too_large")
        self.xs = self.lower[0] + (np.arange(count) + .5) * self.resolution
        self.ys = self.lower[1] + (np.arange(count) + .5) * self.resolution
        xx, yy = np.meshgrid(self.xs, self.ys)
        dx, dy = xx - self.sensor[0], yy - self.sensor[1]
        distance = np.hypot(dx, dy)
        bearing = np.mod(np.arctan2(dy, dx), TAU)
        half_cell = self.resolution / math.sqrt(2.)
        # 對整格的角度範圍取最近回波，而非只檢查格子中心或四個角。
        # 狹窄障礙形成的陰影可能穿過格子中央而不碰到任何一個角。
        angular_half = np.arcsin(np.minimum(1., half_cell / np.maximum(distance, 1e-9)))
        angular_half[distance <= half_cell] = math.pi
        extended_angles = np.concatenate((angles - TAU, angles, angles + TAU))
        extended_lengths = np.tile(lengths, 3)
        left = np.searchsorted(extended_angles, bearing - angular_half, side="right") - 1
        right = np.searchsorted(extended_angles, bearing + angular_half, side="left") + 1
        minimum_range = interval_reduce(extended_lengths, left, right, minimum=True)
        maximum_gap = interval_reduce(np.diff(extended_angles), left, right - 1, minimum=False)
        free = ((maximum_gap <= math.radians(cfg["max_scan_gap_deg"]))
                & (distance + half_cell <= minimum_range))
        # 當前實體車身已經佔有的格子可補為 free；不包含安全 margin。
        local_x = co * (xx - start[0]) + si * (yy - start[1])
        local_y = -si * (xx - start[0]) + co * (yy - start[1])
        physical = ((local_x >= -cfg["rear_overhang_m"] + half_cell)
                    & (local_x <= cfg["body_length_m"] - cfg["rear_overhang_m"] - half_cell)
                    & (np.abs(local_y) <= cfg["body_width_m"] / 2 - half_cell))
        free |= physical
        occupied = np.zeros_like(free)
        endpoints = self.sensor + np.column_stack((np.cos(angles) * lengths, np.sin(angles) * lengths))
        cells = np.floor((endpoints - self.lower) / self.resolution).astype(int)
        inside = ((cells[:, 0] >= 0) & (cells[:, 0] < count) & (cells[:, 1] >= 0) & (cells[:, 1] < count))
        occupied[cells[inside, 1], cells[inside, 0]] = True
        # 車位牆也是障礙；尺寸不能用來推論車位內部必然空曠。
        width, depth = cfg["bay_width_m"], cfg["bay_depth_m"]
        occupied |= (np.abs(xx) <= half_cell) & (np.abs(yy) <= width / 2 + half_cell)
        occupied |= (xx >= -half_cell) & (xx <= depth + half_cell) & (np.abs(np.abs(yy) - width / 2) <= half_cell)
        self.free = free & ~occupied
        self.occupied = occupied
        self.expansion = cfg["margin_m"] + half_cell
        self.half_cell = half_cell
        if time.monotonic() >= deadline:
            raise ValueError("path_timeout")

    def clear(self, pose, extra_padding=0.):
        x, y, yaw = pose
        cfg, pad = self.cfg, self.expansion + extra_padding
        rear = -cfg["rear_overhang_m"] - pad
        front = cfg["body_length_m"] - cfg["rear_overhang_m"] + pad
        side = cfg["body_width_m"] / 2 + pad
        co, si = math.cos(yaw), math.sin(yaw)
        corners = np.array([[rear, -side], [rear, side], [front, -side], [front, side]])
        world_x = x + corners[:, 0] * co - corners[:, 1] * si
        world_y = y + corners[:, 0] * si + corners[:, 1] * co
        i0, i1 = int(math.ceil((min(world_x) - self.lower[0]) / self.resolution - .5)), int(math.floor((max(world_x) - self.lower[0]) / self.resolution - .5))
        j0, j1 = int(math.ceil((min(world_y) - self.lower[1]) / self.resolution - .5)), int(math.floor((max(world_y) - self.lower[1]) / self.resolution - .5))
        if i0 < 0 or j0 < 0 or i1 >= len(self.xs) or j1 >= len(self.ys):
            return False
        dx = self.xs[i0:i1 + 1][None, :] - x
        dy = self.ys[j0:j1 + 1][:, None] - y
        lx, ly = co * dx + si * dy, -si * dx + co * dy
        body = (lx >= rear) & (lx <= front) & (np.abs(ly) <= side)
        return not np.any(body & ~self.free[j0:j1 + 1, i0:i1 + 1])

    def swept(self, pose, gear, curvature, distance):
        divisions = max(1, int(math.ceil(distance / min(self.cfg["sweep_step_m"], self.resolution / 2))))
        # 連續弧線的任一車體點，離最近檢查姿態最多半個步長乘速度上界。
        # 加上此餘量，避免只檢查離散端點而漏過中間擦撞。
        reach = math.hypot(max(self.cfg["rear_overhang_m"], self.cfg["body_length_m"] - self.cfg["rear_overhang_m"])
                           + self.cfg["margin_m"], self.cfg["body_width_m"] / 2 + self.cfg["margin_m"])
        padding = distance / divisions / 2 * (1 + reach * abs(curvature))
        if not self.clear(pose, extra_padding=padding):
            return False
        for index in range(1, divisions + 1):
            if not self.clear(advance(pose, gear * distance * index / divisions, curvature), extra_padding=padding):
                return False
        return True


def plan_path(start_pose, goal_pose, scan_points_body, cfg):
    """只在此次雷達可見區規劃；失敗時 path 永遠是 []，不得帶速重試。

    參數必填 rear_overhang_m / min_turn_radius_m / bay_width_m /
    bay_depth_m / lidar_origin_body_m。budget 上限 3 秒，nodes 上限 20000。
    結果路徑最後一個移動段必須倒車；終點是實際弧線節點而非補假的 goal。
    """
    began = time.monotonic()
    nodes = 0
    def result(ok, reason, path=None, **details):
        value = {"ok": ok, "reason": reason, "path": path or [], "nodes": nodes,
                 "elapsed_s": time.monotonic() - began}
        value.update(details)
        return value
    try:
        settings = _settings(cfg)
        start, goal = _pose(start_pose), _pose(goal_pose)
        if abs(goal[2]) > 1e-6 or abs(goal[1]) > 1e-6:
            return result(False, "path_goal_not_bay_aligned")
        if not 0 < goal[0] < settings["bay_depth_m"]:
            return result(False, "path_goal_outside_bay")
        if settings["bay_width_m"] <= settings["body_width_m"] + 2 * settings["margin_m"]:
            return result(False, "path_bay_too_narrow")
        deadline = began + settings["time_budget_s"]
        grid = VisibleGrid(start, scan_points_body, settings, deadline)
    except (ValueError, TypeError, OverflowError) as exc:
        return result(False, str(exc) if str(exc).startswith("path_") else "path_input_invalid")
    if not grid.clear(start):
        return result(False, "path_start_not_observed_clear")
    if not grid.clear(goal):
        return result(False, "path_goal_not_observed_clear")
    radius, step = settings["min_turn_radius_m"], settings["step_m"]
    yaw_step = math.radians(2.5)
    resolution = min(settings["grid_m"], .05)
    def key(pose, gear):
        return (int(round(pose[0] / resolution)), int(round(pose[1] / resolution)),
                int(round(wrap(pose[2]) / yaw_step)) % 144, gear)
    def heuristic(pose):
        return math.hypot(pose[0] - goal[0], pose[1] - goal[1]) + .3 * radius * abs(wrap(pose[2] - goal[2]))
    def arrived(pose):
        return (math.hypot(pose[0] - goal[0], pose[1] - goal[1]) <= settings["goal_position_tolerance_m"]
                and abs(wrap(pose[2] - goal[2])) <= math.radians(settings["goal_yaw_tolerance_deg"]))
    # records = [pose, gear from parent, curvature from parent, parent index, cost, arc length]
    records = [(start, 0, 0., None, 0., 0.)]
    queue = [(1.8 * heuristic(start), 0)]
    best = {key(start, 0): 0.}
    final = None
    while queue and nodes < settings["max_nodes"]:
        if time.monotonic() >= deadline:
            return result(False, "path_timeout")
        _, index = heapq.heappop(queue)
        pose, gear_before, curvature_before, parent, cost, _ = records[index]
        if cost > best.get(key(pose, gear_before), float("inf")) + 1e-9:
            continue
        nodes += 1
        if gear_before == -1 and arrived(pose):
            final = index
            break
        for gear in (-1, 1):
            actions = [(curvature, step) for curvature in (0., -1. / radius, 1. / radius)]
            angle_error = wrap(goal[2] - pose[2])
            for curvature in (-1. / radius, 1. / radius):
                align_distance = angle_error / (gear * curvature)
                if .015 <= align_distance < step - .001:
                    actions.append((curvature, align_distance))
            # 真正的直線末段，而不是把任意節點直接接到假的終點。
            dx, dy = goal[0] - pose[0], goal[1] - pose[1]
            longitudinal = dx * math.cos(pose[2]) + dy * math.sin(pose[2])
            lateral = -dx * math.sin(pose[2]) + dy * math.cos(pose[2])
            if (gear == -1 and -.3 <= longitudinal < -.005
                    and abs(lateral) < settings["goal_position_tolerance_m"] / 2
                    and abs(angle_error) < math.radians(settings["goal_yaw_tolerance_deg"])):
                actions.insert(0, (0., min(step, -longitudinal)))
            for curvature, arc_length in actions:
                if time.monotonic() >= deadline:
                    return result(False, "path_timeout")
                target = advance(pose, gear * arc_length, curvature)
                changed = gear_before and gear_before != gear
                next_cost = cost + arc_length * (1.0 if gear == -1 else 1.12) + (.55 if changed else 0.)
                next_cost += .015 * abs(curvature) * radius + (.02 if curvature != curvature_before else 0.)
                next_key = key(target, gear)
                terminal = gear == -1 and arrived(target)
                # 短末段可能與尚未到位的父節點落在同一離散 key，不能被父節點
                # 的較低成本抹掉；仍須通過真實弧線掃掠才接受。
                if not terminal and next_cost >= best.get(next_key, float("inf")) - 1e-9:
                    continue
                if not grid.swept(pose, gear, curvature, arc_length):
                    continue
                best[next_key] = next_cost
                records.append((target, gear, curvature, index, next_cost, arc_length))
                child_index = len(records) - 1
                if terminal:
                    final = child_index
                    break
                heapq.heappush(queue, (next_cost + 1.8 * heuristic(target), child_index))
                # 限制待搜尋節點及記憶體，不只限制已展開節點。
                if len(records) >= settings["max_nodes"] * 6:
                    return result(False, "path_node_limit")
            if final is not None:
                break
        if final is not None:
            break
    if final is None:
        return result(False, "path_node_limit" if nodes >= settings["max_nodes"] else "path_no_solution")
    chain = []
    current = final
    while records[current][3] is not None:
        chain.append(current)
        current = records[current][3]
    chain.reverse()
    first_gear = records[chain[0]][1]
    path = [[start[0], start[1], start[2], first_gear, records[chain[0]][2]]]
    for child in chain:
        if time.monotonic() >= deadline:
            return result(False, "path_timeout")
        node_pose, gear, curvature, parent, _, arc_length = records[child]
        previous_pose = records[parent][0]
        divisions = max(1, int(math.ceil(arc_length / settings["sweep_step_m"])))
        for part in range(1, divisions + 1):
            point = advance(previous_pose, gear * arc_length * part / divisions, curvature)
            path.append([point[0], point[1], point[2], gear, curvature])
    finish = records[final][0]
    return result(True, "path_ready", path, goal_error_m=math.hypot(finish[0] - goal[0], finish[1] - goal[1]),
                  goal_error_yaw_rad=abs(wrap(finish[2] - goal[2])), final_gear=-1,
                  length_m=sum(records[child][5] for child in chain), segment_count=len(split_path(path)))


def split_path(path):
    """依移動方向拆段，共用 cusp 位姿；每段完成必須先中立再切下一段。"""
    if not isinstance(path, (list, tuple)) or len(path) < 2:
        raise ValueError("path_points_insufficient")
    points = np.asarray(path, dtype=float)
    if points.ndim != 2 or points.shape[1] != 5 or not np.all(np.isfinite(points)):
        raise ValueError("path_points_invalid")
    if not np.all(np.isin(points[:, 3], [-1, 1])):
        raise ValueError("path_gear_invalid")
    segments = []
    segment = [list(points[0])]
    segment[0][3] = int(points[1, 3])
    segment[0][4] = float(points[1, 4])
    previous_gear = int(points[1, 3])
    for index in range(1, len(points)):
        point = list(points[index])
        point[3] = int(point[3])
        gear = point[3]
        if gear != previous_gear:
            segments.append(segment)
            cusp = list(points[index - 1])
            cusp[3], cusp[4] = gear, point[4]
            segment = [cusp]
        segment.append(point)
        previous_gear = gear
    segments.append(segment)
    return segments


def follow_path(path, pose, previous_progress_m=0., lookahead_m=.25, cfg=None):
    """追蹤單一 gear segment。進度來自公尺投影，完成只代表該段到位。

    倒車時 signed_curvature 仍是實體轉向曲率；gear * signed_curvature 才是
    單位正行駛距離的 yaw 變化。此函式不跨 cusp、不自動切到另一個 gear。
    """
    options = {} if cfg is None else cfg
    failure = {"ok": False, "completed": False, "stop": True, "gear_change": False}
    try:
        if not isinstance(options, dict):
            raise ValueError("path_config_invalid")
        points = np.asarray(path, dtype=float)
        current = _pose(pose)
        if points.ndim != 2 or points.shape[1] != 5 or len(points) < 2 or not np.all(np.isfinite(points)):
            raise ValueError("path_points_invalid")
        gears = np.unique(points[:, 3])
        if len(gears) != 1 or gears[0] not in (-1, 1):
            raise ValueError("path_follow_requires_single_gear")
        if not finite_number(previous_progress_m) or previous_progress_m < 0 or not finite_number(lookahead_m) or not .08 <= lookahead_m <= .6:
            raise ValueError("path_follow_progress_invalid")
        radius = options.get("min_turn_radius_m")
        if not finite_number(radius) or not .25 <= radius <= 4:
            raise ValueError("path_config_missing:min_turn_radius_m")
        delta = points[1:, :2] - points[:-1, :2]
        lengths = np.linalg.norm(delta, axis=1)
        if np.any(lengths < 1e-6) or np.any(lengths > .4):
            raise ValueError("path_follow_spacing_invalid")
        cumulative = np.concatenate(([0.], np.cumsum(lengths)))
        total = float(cumulative[-1])
        if previous_progress_m > total + .05:
            raise ValueError("path_follow_progress_invalid")
        window = float(options.get("max_progress_step_m", .5))
        if not .1 <= window <= 1.:
            raise ValueError("path_follow_window_invalid")
        fractions = np.sum((np.asarray(current[:2]) - points[:-1, :2]) * delta, axis=1) / lengths ** 2
        fractions = np.clip(fractions, 0., 1.)
        projected = points[:-1, :2] + fractions[:, None] * delta
        distance = np.linalg.norm(projected - np.asarray(current[:2]), axis=1)
        progress_values = cumulative[:-1] + fractions * lengths
        eligible = ((progress_values >= max(0., previous_progress_m - .08))
                    & (progress_values <= previous_progress_m + window))
        if not np.any(eligible):
            raise ValueError("path_follow_no_projection")
        distance[~eligible] = float("inf")
        nearest = int(np.argmin(distance))
        max_error = options.get("max_cross_track_m", options.get("max_tracking_error_m", .18))
        if not finite_number(max_error) or not .05 <= max_error <= .25:
            raise ValueError("path_follow_tracking_limit_invalid")
        if distance[nearest] > max_error:
            raise ValueError("path_follow_deviation")
        progress = max(float(previous_progress_m), float(progress_values[nearest]))
        end_error = math.hypot(current[0] - points[-1, 0], current[1] - points[-1, 1])
        yaw_error = abs(wrap(current[2] - float(points[-1, 2])))
        position_tolerance = options.get("position_tolerance_m", .08)
        yaw_tolerance = options.get("yaw_tolerance_deg", 6.)
        if (not finite_number(position_tolerance) or not .04 <= position_tolerance <= .12
                or not finite_number(yaw_tolerance) or not 2 <= yaw_tolerance <= 8):
            raise ValueError("path_follow_tolerance_invalid")
        completed = (total - progress <= .4 * position_tolerance
                     and end_error <= .4 * position_tolerance and yaw_error <= math.radians(.4 * yaw_tolerance))
        target_progress = min(total, progress + lookahead_m)
        index = min(len(lengths) - 1, int(np.searchsorted(cumulative, target_progress, side="right") - 1))
        fraction = (target_progress - cumulative[index]) / lengths[index]
        target_xy = points[index, :2] + fraction * delta[index]
        target_yaw = wrap(float(points[index, 2]) + fraction * wrap(float(points[index + 1, 2] - points[index, 2])))
        dx, dy = target_xy[0] - current[0], target_xy[1] - current[1]
        local_x = math.cos(current[2]) * dx + math.sin(current[2]) * dy
        local_y = -math.sin(current[2]) * dx + math.cos(current[2]) * dy
        gear = int(gears[0])
        if not completed and (gear * local_x < -.02 or dx * dx + dy * dy < 1e-8):
            raise ValueError("path_follow_target_behind_motion")
        curvature = 0. if completed else float(2 * local_y / (dx * dx + dy * dy))
        curvature = max(-1. / radius, min(1. / radius, curvature))
        return {"ok": True, "reason": "path_segment_complete" if completed else "path_tracking",
                "completed": completed, "stop": completed, "gear_change": False,
                "gear": gear, "signed_curvature": curvature, "progress_m": progress,
                "remaining_m": max(0., total - progress), "tracking_error_m": float(distance[nearest]),
                "target": [float(target_xy[0]), float(target_xy[1]), target_yaw],
                "yaw_change_per_travel_m": gear * curvature}
    except (ValueError, TypeError, OverflowError) as exc:
        failure["reason"] = str(exc) if str(exc).startswith("path_") else "path_follow_input_invalid"
        return failure
