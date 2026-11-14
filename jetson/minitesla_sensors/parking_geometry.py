"""由實體 U 形牆面的雷達回波估算車位與車身相對姿態。

輸入：後軸中心原點、x 向前、y 向左的公尺座標。車位後牆 x=0，
左右牆 y=±width/2，入口 x=depth。輸出的 pose 是後軸在車位座標內
的 [x, y, yaw]；yaw 逆時針為正。缺回波不當作可通行空間。

這是幾何觀測器，不由馬達命令或經過時間推算車位。無足夠不同方向
牆面時回傳無效；輸出仍須由控制層檢查資料時效、連續性與碰撞。
"""
import math

import numpy as np


def _options(cfg):
    cfg = cfg or {}
    values = {
        'width': float(cfg.get('bay_width_m', 1.6)),
        'depth': float(cfg.get('bay_depth_m', 1.8)),
        'dimension_tol': float(cfg.get('bay_dimension_tolerance_m', 0.15)),
        'line_tol': float(cfg.get('geometry_line_tolerance_m', 0.035)),
        'angle_tol': math.radians(float(cfg.get('geometry_angle_tolerance_deg', 8))),
        'min_span': float(cfg.get('geometry_min_line_span_m', 0.30)),
        'min_support': int(cfg.get('geometry_min_line_support', 9)),
        'max_jump': float(cfg.get('geometry_track_max_jump_m', 0.35)),
        'max_yaw_jump': math.radians(float(cfg.get('geometry_track_max_yaw_deg', 12))),
    }
    if (not all(math.isfinite(value) and value > 0 for value in values.values()) or
            values['dimension_tol'] >= min(values['width'], values['depth']) / 3 or
            values['min_support'] < 4 or values['angle_tol'] >= math.pi / 4):
        raise ValueError('停車幾何設定超出有效範圍')
    origin = np.asarray(cfg.get('lidar_origin_body_m', [0.0, 0.0]), dtype=float)
    if origin.shape != (2,) or not np.isfinite(origin).all():
        raise ValueError('lidar_origin_body_m 必須是有限的 [x, y] 公尺座標')
    values['lidar_origin'] = origin
    return values


def _points(points):
    points = np.asarray(points, dtype=float)
    if points.size == 0:
        return np.empty((0, 2))
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError('雷達點必須是 N×2 座標')
    points = points[np.isfinite(points).all(axis=1)]
    points = points[np.linalg.norm(points, axis=1) <= 15.0]
    if not len(points):
        return points
    # 固定空間格可避免同一方向重複資料支配線段，也限制 RANSAC 工作量。
    _, indexes = np.unique(np.floor(points / 0.018).astype(np.int64),
                           axis=0, return_index=True)
    points = points[np.sort(indexes)]
    if len(points) > 1800:
        points = points[np.linspace(0, len(points)-1, 1800, dtype=int)]
    return points


def _fit_line(points):
    center = np.mean(points, axis=0)
    _, _, vectors = np.linalg.svd(points-center, full_matrices=False)
    axis = vectors[0]
    if axis[0] < 0 or (abs(axis[0]) < 1e-12 and axis[1] < 0):
        axis = -axis
    normal = np.array([-axis[1], axis[0]])
    projection = np.dot(points-center, axis)
    return {'center': center, 'axis': axis, 'points': points,
            'span': float(np.ptp(projection)),
            'error': float(np.sqrt(np.mean(np.dot(points-center, normal)**2)))}


def _lines(points, opt):
    remaining = points.copy()
    random = np.random.RandomState(9147)
    lines = []
    for _ in range(16):
        if len(remaining) < opt['min_support']:
            break
        best = None
        best_count = opt['min_support']-1
        for unused in range(160):
            pair = random.choice(len(remaining), 2, replace=False)
            delta = remaining[pair[1]]-remaining[pair[0]]
            length = np.linalg.norm(delta)
            if length < opt['min_span']:
                continue
            normal = np.array([-delta[1], delta[0]]) / length
            mask = np.abs(np.dot(remaining-remaining[pair[0]], normal)) <= opt['line_tol']
            count = int(np.count_nonzero(mask))
            if count > best_count:
                best_count, best = count, mask
        if best is None:
            break
        for unused in range(2):
            line = _fit_line(remaining[best])
            normal = np.array([-line['axis'][1], line['axis'][0]])
            best = np.abs(np.dot(remaining-line['center'], normal)) <= opt['line_tol']
        inliers = remaining[best]
        remaining = remaining[~best]
        if len(inliers) < opt['min_support']:
            continue
        line = _fit_line(inliers)
        projection = np.dot(inliers, line['axis'])
        order = np.argsort(projection)
        # 長缺口保留為不同線段，避免把隔壁車位或門洞接成同一面牆。
        splits = np.where(np.diff(projection[order]) > 0.28)[0]+1
        for indexes in np.split(order, splits):
            if len(indexes) < opt['min_support']:
                continue
            segment = _fit_line(inliers[indexes])
            if segment['span'] >= opt['min_span']:
                lines.append(segment)
        if len(lines) >= 20:
            break
    return sorted(lines, key=lambda item: -len(item['points']))[:20]


def _angle_difference(a, b):
    return abs(math.atan2(math.sin(a-b), math.cos(a-b)))


def _axes(back, sides, preferred=None):
    direction = sides[0]['axis'].copy()
    if preferred is not None and np.dot(direction, preferred) < 0:
        direction = -direction
    vectors = [(side['axis'], len(side['points'])) for side in sides]
    vectors.append((np.array([-back['axis'][1], back['axis'][0]]), len(back['points'])))
    result = np.zeros(2)
    for vector, weight in vectors:
        result += (vector if np.dot(vector, direction) >= 0 else -vector) * weight
    result /= np.linalg.norm(result)
    if preferred is None:
        toward_sides = np.mean([side['center'] for side in sides], axis=0)-back['center']
        if np.dot(result, toward_sides) < 0:
            result = -result
    return result, np.array([-result[1], result[0]])


def _opening_visible(points, pose, opt, width, depth):
    """只計入實際回波之前穿過入口的自由射線；無回波不計分。"""
    x, y, yaw = pose
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    ends = np.dot(points, rotation.T) + [x, y]
    origin = np.dot(rotation, opt['lidar_origin']) + [x, y]
    occupied_entry = ends[(np.abs(ends[:, 0]-depth) <= 0.10) &
                          (np.abs(ends[:, 1]) < width/2-0.08)]
    if (len(occupied_entry) >= opt['min_support'] and
            np.ptp(occupied_entry[:, 1]) >= width*0.35):
        # 完整或大幅封住入口的第四面牆，不能被少量遠端雜點宣告為空。
        return False
    delta = ends-origin
    with np.errstate(divide='ignore', invalid='ignore'):
        fraction = (depth-origin[0]) / delta[:, 0]
        crossings = origin[1] + fraction*delta[:, 1]
    # 入口前後都需至少 8 公分的實際可見射線，不接受正好撞上入口牆的回波。
    outside = (origin[0] > depth+0.08) & (ends[:, 0] < depth-0.08)
    inside = (origin[0] < depth-0.08) & (ends[:, 0] > depth+0.08)
    selected = ((outside | inside) & (fraction > 0) & (fraction < 1) &
                np.isfinite(crossings) & (np.abs(crossings) < width/2-0.08))
    crossings = crossings[selected]
    return len(crossings) >= 8 and np.ptp(crossings) >= width*0.4


def _candidate(pose, width, depth, back, sides, fit_error, opening_visible=False):
    x, y, yaw = [float(value) for value in pose]
    support = len(back['points'])+sum(len(side['points']) for side in sides)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    origin = -np.dot(rotation.T, [x, y])
    return {'pose': [x, y, yaw], 'width_m': float(width), 'depth_m': float(depth),
            'score': float(min(1.0, support/70.0)*math.exp(-fit_error/0.04)),
            'fit_error_m': float(fit_error), 'support': int(support),
            'wall_count': 1+len(sides), 'opening_visible': bool(opening_visible),
            'origin_body_m': origin.tolist(),
            'bay_axes_body': {'x': rotation[0].tolist(), 'y': rotation[1].tolist()}}


def _deduplicate(candidates):
    result = []
    for candidate in sorted(candidates, key=lambda c: (c['wall_count'], c['score']), reverse=True):
        if any(np.linalg.norm(np.asarray(candidate['pose'][:2])-other['pose'][:2]) < 0.06 and
               _angle_difference(candidate['pose'][2], other['pose'][2]) < math.radians(3)
               for other in result):
            continue
        result.append(candidate)
    return result


def detect_bays(points, cfg=None):
    """初次辨識完整 U 形車位；傳回所有分離候選，交由上層拒絕歧義。"""
    opt = _options(cfg)
    points = _points(points)
    lines = _lines(points, opt)
    candidates = []
    cosine = math.cos(opt['angle_tol'])
    sine = math.sin(opt['angle_tol'])
    tolerance = opt['dimension_tol']
    for first in range(len(lines)):
        for second in range(first+1, len(lines)):
            sides = [lines[first], lines[second]]
            if abs(np.dot(sides[0]['axis'], sides[1]['axis'])) < cosine:
                continue
            for index, back in enumerate(lines):
                if index in (first, second):
                    continue
                if any(abs(np.dot(back['axis'], side['axis'])) > sine for side in sides):
                    continue
                xaxis, yaxis = _axes(back, sides)
                bx = float(np.mean(np.dot(back['points'], xaxis)))
                centers = sorted(float(np.mean(np.dot(side['points'], yaxis))) for side in sides)
                width = centers[1]-centers[0]
                if abs(width-opt['width']) > tolerance:
                    continue
                cy = (centers[0]+centers[1])/2
                extents = [(float(np.min(np.dot(side['points'], xaxis))-bx),
                            float(np.max(np.dot(side['points'], xaxis))-bx)) for side in sides]
                # 一度雷達分箱在牆角的擦邊射線可能相隔十多公分。
                # 後牆位置由實際後牆回波決定；側牆近角允許兩倍取樣缺口，
                # 入口端的車位深度仍必須符合原本的尺寸容差。
                if any(low < -tolerance or low > 2*tolerance or
                       abs(high-opt['depth']) > tolerance
                       for low, high in extents):
                    continue
                back_y = np.dot(back['points'], yaxis)
                # 從入口斜看，後牆角落可被實體側牆遮住；寬度由兩側牆
                # 間距量得，但後牆仍須有至少 45% 寬度的實際同一直線回波。
                if (float(np.min(back_y)) < centers[0]-tolerance or
                        float(np.max(back_y)) > centers[1]+tolerance or
                        float(np.ptp(back_y)) < width*0.45):
                    continue
                depth = sum(high for low, high in extents)/2
                pose = [-bx, -cy, math.atan2(-xaxis[1], xaxis[0])]
                if not _opening_visible(points, pose, opt, width, depth):
                    continue
                residuals = list(np.dot(back['points'], xaxis)-bx)
                for side in sides:
                    side_y = np.dot(side['points'], yaxis)
                    residuals.extend(side_y-np.mean(side_y))
                error = float(np.sqrt(np.mean(np.asarray(residuals)**2)))
                if error > opt['line_tol']:
                    continue
                candidates.append(_candidate(pose, width, depth, back, sides, error, True))
    return _deduplicate(candidates)


def track_bay(points, cfg, previous_pose):
    """從本幀後牆＋側牆重新估位；單靠平行牆、舊姿態或空回波皆無效。"""
    opt = _options(cfg)
    prior = np.asarray(previous_pose, dtype=float)
    if prior.shape != (3,) or not np.isfinite(prior).all():
        return None
    points = _points(points)
    lines = _lines(points, opt)
    cosine, sine = math.cos(prior[2]), math.sin(prior[2])
    old_x = np.array([cosine, -sine])
    old_y = np.array([sine, cosine])
    back_matches, left_matches, right_matches = [], [], []
    for line in lines:
        x = np.dot(line['points'], old_x)+prior[0]
        y = np.dot(line['points'], old_y)+prior[1]
        if (abs(np.dot(line['axis'], old_x)) < math.sin(opt['angle_tol']) and
                abs(float(np.mean(x))) <= opt['max_jump'] and
                np.min(y) >= -opt['width']/2-opt['dimension_tol'] and
                np.max(y) <= opt['width']/2+opt['dimension_tol']):
            back_matches.append(line)
        if (abs(np.dot(line['axis'], old_x)) > math.cos(opt['angle_tol']) and
                np.min(x) >= -opt['dimension_tol']-opt['max_jump'] and
                np.max(x) <= opt['depth']+opt['dimension_tol']+opt['max_jump']):
            center = float(np.mean(y))
            if abs(center-opt['width']/2) <= opt['max_jump']:
                left_matches.append(line)
            if abs(center+opt['width']/2) <= opt['max_jump']:
                right_matches.append(line)
    candidates = []
    for back in back_matches:
        combinations = [([line], [opt['width']/2]) for line in left_matches]
        combinations += [([line], [-opt['width']/2]) for line in right_matches]
        combinations += [([left, right], [opt['width']/2, -opt['width']/2])
                         for left in left_matches for right in right_matches]
        for sides, offsets in combinations:
            if any(abs(np.dot(back['axis'], side['axis'])) > math.sin(opt['angle_tol'])
                   for side in sides):
                continue
            xaxis, yaxis = _axes(back, sides, old_x)
            tx = -float(np.mean(np.dot(back['points'], xaxis)))
            weights = np.array([len(side['points']) for side in sides], dtype=float)
            translations = [offset-float(np.mean(np.dot(side['points'], yaxis)))
                            for side, offset in zip(sides, offsets)]
            ty = float(np.average(translations, weights=weights))
            yaw = math.atan2(-xaxis[1], xaxis[0])
            if (np.linalg.norm(np.array([tx, ty])-prior[:2]) > opt['max_jump'] or
                    _angle_difference(yaw, prior[2]) > opt['max_yaw_jump']):
                continue
            # 追蹤也保留至少一側牆到入口端的深度證據；若新障礙物遮住
            # 真正後牆，不可僅把較近的平行障礙物重新命名成原本的後牆。
            extents = [(float(np.min(np.dot(side['points'], xaxis))+tx),
                        float(np.max(np.dot(side['points'], xaxis))+tx)) for side in sides]
            if (any(low < -opt['dimension_tol'] or high > opt['depth']+opt['dimension_tol']
                    for low, high in extents) or
                    not any(abs(high-opt['depth']) <= opt['dimension_tol']
                            for low, high in extents)):
                continue
            back_y = np.dot(back['points'], yaxis)+ty
            if (np.min(back_y) < -opt['width']/2-opt['dimension_tol'] or
                    np.max(back_y) > opt['width']/2+opt['dimension_tol']):
                continue
            residuals = list(np.dot(back['points'], xaxis)+tx)
            for side, offset in zip(sides, offsets):
                residuals.extend(np.dot(side['points'], yaxis)+ty-offset)
            error = float(np.sqrt(np.mean(np.asarray(residuals)**2)))
            if error > opt['line_tol']:
                continue
            candidates.append(_candidate([tx, ty, yaw], opt['width'], opt['depth'],
                                         back, sides, error))
    candidates = _deduplicate(candidates)
    # 同一前次姿態附近仍有多個相異解，不能任意選分數最高者。
    if len(candidates) != 1:
        return None
    return candidates[0]
