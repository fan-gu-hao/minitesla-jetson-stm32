#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""設定及唯讀停車預覽；本工具不開啟 UART，也不會發出行駛命令。"""
import argparse
import html
import json
import math
import os
from pathlib import Path
import sys
import time

from minitesla_sensors.parking_config import defaults, validate_settings, load_settings

ROOT = Path(__file__).resolve().parent
REQUIRED = {
    "lidar_yaw_deg": "雷達原始零度對應的車體角度（向前0°、向右90°）",
    "lidar_y_from_center_m": "雷達距車身中心線的橫向位置（公尺，右正左負）",
    "rear_overhang_m": "車尾至後軸中心距離（公尺）",
    "left_turn_radius_m": "左轉命令下後軸中心的轉彎半徑（公尺）",
    "right_turn_radius_m": "右轉命令下後軸中心的轉彎半徑（公尺）",
}


def save(path, cfg):
    cleaned = {k: v for k, v in cfg.items() if k in defaults()}
    validate_settings(cleaned)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + '.tmp')
    with temp.open('w', encoding='utf-8') as f:
        json.dump(cleaned, f, ensure_ascii=False, allow_nan=False, indent=2)
        f.write('\n')
        f.flush()
        os.fsync(f.fileno())
    os.replace(str(temp), str(path))


def status(cfg):
    print('自動找位入庫：' + ('已啟用' if cfg['enabled'] else '未啟用'))
    for key, label in REQUIRED.items():
        print('{}：{}'.format(label, cfg[key] if cfg[key] is not None else '尚未設定'))
    print('雷達角度遞增方向：' + ('順時針' if cfg['lidar_clockwise'] else '逆時針'))
    print('車位寬／深：{:.3f}／{:.3f} 公尺'.format(cfg['bay_width_m'], cfg['bay_depth_m']))
    print('轉向命令：左 {}／中 152／右 {}'.format(cfg['steering_left_pwm'], cfg['steering_right_pwm']))
    print('設定儲存後，需重新啟動車端服務才會生效。')


def interactive(cfg):
    print('依既有安裝與轉向資料填寫。直接 Enter 保留現值；本工具不會移動車子。')
    text = input('雷達原始角度是否順時針遞增？[y/n；Enter保留]：').strip().lower()
    if text:
        if text not in ('y', 'n'):
            raise ValueError('方向請輸入 y 或 n')
        clockwise = text == 'y'
        if clockwise != cfg['lidar_clockwise']:
            cfg['lidar_yaw_deg'] = None
            print('已變更掃描方向，請重新確認雷達朝向角度。')
        cfg['lidar_clockwise'] = clockwise
    for key, label in REQUIRED.items():
        current = cfg.get(key)
        value = input('{} [{}]：'.format(label, '' if current is None else current)).strip()
        if value:
            cfg[key] = float(value)
    for key, label in (('bay_width_m', '實體車位內寬'), ('bay_depth_m', '實體車位內深')):
        value = input('{}，公尺 [{}]：'.format(label, cfg[key])).strip()
        if value:
            cfg[key] = float(value)
    cfg['enabled'] = False
    return validate_settings(cfg)


def current_snapshot(path):
    if path.stat().st_size > 5 * 1024 * 1024:
        raise ValueError('感測快照過大，請確認檔案')
    with path.open(encoding='utf-8') as f:
        snap = json.load(f)
    now = time.monotonic()
    stamp = snap.get('monotonic_s')
    if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
        raise ValueError('快照缺少有效時間')
    if not 0 <= now-stamp <= 1.0:
        raise ValueError('快照已過期；請保持車端主程式運作，並使用同一台 Jetson 執行預覽')
    bootfile = Path('/proc/sys/kernel/random/boot_id')
    if bootfile.exists() and snap.get('boot_id') != bootfile.read_text().strip():
        raise ValueError('快照不是本次開機產生')
    if snap.get('runtime', {}).get('state') != 'running':
        raise ValueError('車端主程式目前未運作')
    if snap.get('control', {}).get('mode') != 'standby':
        raise ValueError('先在電腦操作台按空白鍵，確認待命後再產生預覽')
    return snap, now


def render_preview(path, points, pose, candidates, planned, cfg):
    import numpy as np
    # 轉為車位座標；圖上朝出口+x在上方，+y在左方。
    c, s = math.cos(pose[2]), math.sin(pose[2])
    rotation = np.array([[c, -s], [s, c]])
    world = points.dot(rotation.T) + np.asarray(pose[:2])
    draw_points = world[::max(1, len(world)//450)]
    path_points = planned.get('path', [])
    vertices = [(0, -cfg['bay_width_m']/2), (cfg['bay_depth_m'], -cfg['bay_width_m']/2),
                (0, cfg['bay_width_m']/2), (cfg['bay_depth_m'], cfg['bay_width_m']/2), tuple(pose[:2])]
    vertices += [(p[0],p[1]) for p in path_points]
    min_x, max_x = min(p[0] for p in vertices)-.6, max(p[0] for p in vertices)+.6
    min_y, max_y = min(p[1] for p in vertices)-.6, max(p[1] for p in vertices)+.6
    scale = min(740/(max_y-min_y), 580/(max_x-min_x))
    def xy(x,y):
        return 80+(max_y-y)*scale, 110+(max_x-x)*scale
    def line(a,b,color,width=3,dash=''):
        u,v=xy(*a); w,z=xy(*b)
        return '<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" stroke="%s" stroke-width="%s" %s/>'%(u,v,w,z,color,width,dash)
    title='停車路徑預覽（不會移動車輛）'
    svg=['<svg xmlns="http://www.w3.org/2000/svg" width="960" height="820" viewBox="0 0 960 820">',
         '<rect width="960" height="820" fill="#0f172a"/>',
         '<g font-family="Microsoft JhengHei,Noto Sans CJK TC,sans-serif" fill="#f8fafc">',
         '<text x="32" y="44" font-size="25">'+title+'</text>',
         '<text x="32" y="74" font-size="16">灰點：實際回波　青線：前進　紫線：倒車　橘框：目前車體</text>']
    for x,y in draw_points:
        u,v=xy(x,y)
        if 15<u<945 and 95<v<725:
            svg.append('<circle cx="%.2f" cy="%.2f" r="1.7" fill="#64748b"/>'%(u,v))
    w,d=cfg['bay_width_m']/2,cfg['bay_depth_m']
    for a,b in [((0,-w),(0,w)),((0,-w),(d,-w)),((0,w),(d,w))]:
        svg.append(line(a,b,'#e2e8f0',4))
    svg.append(line((d,-w),(d,w),'#94a3b8',1,'stroke-dasharray="6 6"'))
    for a,b in zip(path_points,path_points[1:]):
        svg.append(line(a[:2],b[:2],'#22d3ee' if b[3]>0 else '#c084fc',3))
    rear,front=cfg['rear_overhang_m'],cfg['body_length_m']-cfg['rear_overhang_m']
    corners=np.array([[-rear,-cfg['body_width_m']/2],[front,-cfg['body_width_m']/2],
                      [front,cfg['body_width_m']/2],[-rear,cfg['body_width_m']/2]])
    corners=corners.dot(rotation.T)+np.array(pose[:2])
    svg.append('<polygon points="'+' '.join('%.2f,%.2f'%xy(*p) for p in corners)+'" fill="#fb923c33" stroke="#fb923c" stroke-width="3"/>')
    reason='找到可行路徑' if planned.get('ok') else '尚未找到可行路徑：'+str(planned.get('reason','未知'))
    for i,text in enumerate([reason,'辨識車位 %d 個；圖中為距入口最近的候選。'%len(candidates),
                            '預覽只是當下資料；正式入庫會重新辨識、定位與規劃。']):
        svg.append('<text x="32" y="%d" font-size="16">%s</text>'%(745+i*25,html.escape(text)))
    svg.append('</g></svg>')
    path.write_text('\n'.join(svg),encoding='utf-8')


def preview(args,cfg):
    # 暫時驗證完整參數，不改啟用狀態、不送任何控制封包。
    cfg=dict(cfg,enabled=True)
    cfg=validate_settings(cfg,require_enabled=True)
    from minitesla_sensors.parking_auto import extract_scan
    from minitesla_sensors.parking_geometry import detect_bays
    from minitesla_sensors.parking_path import plan_path
    snap,now=current_snapshot(Path(args.snapshot).expanduser())
    scan=extract_scan(snap,cfg,now)
    if scan is None:
        raise ValueError('沒有足夠新鮮雷達回波；稍後重試並確認雷達運作')
    candidates=detect_bays(scan['points'],cfg)
    if not candidates:
        raise ValueError('未辨識到尺寸相符且入口可見的三面實體車位；請確認擋板高於雷達掃描平面、寬深與方向設定')
    candidates.sort(key=lambda b:math.hypot(b['pose'][0]-cfg['bay_depth_m'],b['pose'][1]))
    pose=candidates[0]['pose']
    goal=[cfg['target_back_clearance_m']+cfg['rear_overhang_m'],0,0]
    planned=plan_path(pose,goal,scan['points'],cfg)
    target=Path(args.output).expanduser()
    target.parent.mkdir(parents=True,exist_ok=True)
    render_preview(target,scan['points'],pose,candidates,planned,cfg)
    print('車位相對位置：x={:.3f}m，y={:.3f}m，車身角度={:.1f}°'.format(pose[0],pose[1],math.degrees(pose[2])))
    print('路徑：'+('可行' if planned.get('ok') else '無可行解（'+str(planned.get('reason'))+'）'))
    print('預覽檔：'+str(target.resolve()))
    print('這個預覽沒有啟用停車，也沒有發出油門或轉向命令。')
    return 0 if planned.get('ok') else 2


def main(argv=None):
    parser=argparse.ArgumentParser(description='自動停車設定與唯讀路徑預覽')
    parser.add_argument('--config',default=str(ROOT/'parking_config.json'))
    sub=parser.add_subparsers(dest='action')
    sub.add_parser('status'); sub.add_parser('init'); sub.add_parser('configure')
    sub.add_parser('enable'); sub.add_parser('disable')
    setcmd=sub.add_parser('set')
    for name in ('front-angle','lidar-x-from-tail-mm','lidar-y-from-center-mm','rear-overhang-mm',
                 'left-radius-mm','right-radius-mm','bay-width-mm','bay-depth-mm'):
        setcmd.add_argument('--'+name,type=float)
    setcmd.add_argument('--left-pwm',type=int); setcmd.add_argument('--right-pwm',type=int)
    direction=setcmd.add_mutually_exclusive_group()
    direction.add_argument('--clockwise',dest='clockwise',action='store_true')
    direction.add_argument('--counterclockwise',dest='clockwise',action='store_false')
    setcmd.set_defaults(clockwise=None)
    show=sub.add_parser('preview')
    show.add_argument('--snapshot',default=str(ROOT/'sensor_snapshot.json'))
    show.add_argument('--output',default=str(ROOT/'parking_preview.svg'))
    args=parser.parse_args(argv)
    action=args.action or 'status'
    path=Path(args.config).expanduser()
    try:
        if action=='init':
            if path.exists():
                raise ValueError('設定檔已存在，未覆寫；請使用 configure 或 set')
            save(path,defaults()); print('已建立未啟用的停車設定：'+str(path)); return 0
        cfg=load_settings(path)
        if action=='status': status(cfg); return 0
        if action=='preview': return preview(args,cfg)
        if action=='configure': cfg=interactive(cfg)
        elif action=='set':
            if args.clockwise is not None:
                if args.clockwise != cfg['lidar_clockwise'] and args.front_angle is None:
                    cfg['lidar_yaw_deg'] = None
                cfg['lidar_clockwise']=args.clockwise
            if args.front_angle is not None:
                if not math.isfinite(args.front_angle) or not 0<=args.front_angle<360:
                    raise ValueError('正前方原始角度必須介於0（含）與360（不含）度')
                sign=1 if cfg['lidar_clockwise'] else -1
                cfg['lidar_yaw_deg']=(((-sign*args.front_angle)+180)%360)-180
            for source,key in [('lidar_x_from_tail_mm','lidar_x_from_tail_m'),('lidar_y_from_center_mm','lidar_y_from_center_m'),
                               ('rear_overhang_mm','rear_overhang_m'),('left_radius_mm','left_turn_radius_m'),
                               ('right_radius_mm','right_turn_radius_m'),('bay_width_mm','bay_width_m'),('bay_depth_mm','bay_depth_m')]:
                value=getattr(args,source)
                if value is not None: cfg[key]=value/1000
            for source,key in [('left_pwm','steering_left_pwm'),('right_pwm','steering_right_pwm')]:
                value=getattr(args,source)
                if value is not None: cfg[key]=value
            cfg['enabled']=False  # 修改幾何後需明確重新啟用，避免沿用舊路徑假設。
        elif action=='enable': cfg['enabled']=True
        elif action=='disable': cfg['enabled']=False
        cfg=validate_settings(cfg)
        save(path,cfg); status(cfg)
        return 0
    except (OSError,ValueError,EOFError,KeyboardInterrupt) as exc:
        print('設定未完成：'+str(exc),file=sys.stderr)
        return 2


if __name__=='__main__':
    raise SystemExit(main())
