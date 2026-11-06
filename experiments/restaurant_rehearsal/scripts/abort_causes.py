"""ABORTED 원인 분류: system 로그 + (있으면) rosbag. 컨테이너에서 돈다 (rosbag2_py).

    python3 abort_causes.py <raw 폴더> [<raw 폴더> ...] --map <restaurant_gt.yaml>

실행마다:
- 로그: 안전 노드 정지(safety WARNING/CRITICAL·근접 저속), DWA 궤적 없음, 진행 없음(progress checker), 복구 실패(spin·backup
  "Collision Ahead" 등), 경로 계획 실패
- 정지 구간(로봇 GT 속력 < 0.03 m/s가 3 s 이상): 구간마다 가장 가까운 손님과 거리, 그 손님이 그동안 움직였는지(비키지 않고
  지나가거나 다가옴), 로봇 자리의 정적 지도 여유(서빙 로봇 내접 반경 0.27 m + 팽창이 통로 폭보다 큰지)
"""
import argparse
import json
import math
import re
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

HALF_L, HALF_W = 0.565 / 2, 0.537 / 2
INSCRIBED = min(HALF_L, HALF_W)
CIRCUMSCRIBED = math.hypot(HALF_L, HALF_W)
GUEST_R = 0.3          # 손님 몸 반지름 (customer 모델 충돌체 근사)
STOP_V, STOP_S = 0.03, 3.0

PATTERNS = {
    'safety_stop': r'amr_safety|safety_node.*(WARNING|CRITICAL|정지|저속)',
    'dwa_no_traj': r'DWA: 유효한 궤적을 못 찾아',
    'no_progress': r'Failed to make progress',
    'recovery_fail': r'(spin|backup|drive_on_heading) failed|Collision Ahead',
    'plan_fail': r'Failed to create plan|GridBased plugin failed|planner .*failed',
    'goal_aborted': r'Goal failed|Aborting|aborted',
}


def log_counts(path):
    text = path.read_text(encoding='utf-8', errors='replace') if path.exists() else ''
    return {k: len(re.findall(p, text)) for k, p in PATTERNS.items()}


def load_map(yaml_path):
    m = yaml.safe_load(open(yaml_path))
    img = np.array(Image.open(Path(yaml_path).parent / m['image']))
    occ = img < 128
    dist = ndimage.distance_transform_edt(~occ) * m['resolution']
    return dist, m['resolution'], m['origin'][:2], img.shape[0]


def clearance(dmap, x, y):
    dist, res, (ox, oy), h = dmap
    r, c = h - 1 - int((y - oy) / res), int((x - ox) / res)
    return float(dist[r, c]) if 0 <= r < dist.shape[0] and 0 <= c < dist.shape[1] else float('nan')


def read_bag(bag_dir):
    import rosbag2_py
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    robot, guests, cmd = [], {}, []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic == '/amr_01/cmd_vel':
            c = deserialize_message(data, Twist)
            cmd.append((stamp * 1e-9, abs(c.linear.x) + abs(c.angular.z)))
            continue
        if not topic.endswith('ground_truth/odom'):
            continue
        m = deserialize_message(data, Odometry)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = (t, m.pose.pose.position.x, m.pose.pose.position.y,
             math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y))
        if topic.startswith('/amr_01/'):
            robot.append(p + (stamp * 1e-9,))
        elif topic.startswith('/dynamic_obstacles/'):
            guests.setdefault(topic.split('/')[2], []).append(p)
    return sorted(robot), {k: sorted(v) for k, v in guests.items()}, sorted(cmd)


def pushed(robot, cmd):
    """명령이 0인데(마지막 cmd_vel |v|+|w| < 0.01) GT 속력 > 0.05 m/s로 움직인 시간·거리 = 손님에게 밀림. 시각은 bag 수신 시각."""
    if not cmd:
        return None
    import bisect
    ts = [c[0] for c in cmd]
    dur = dist = 0.0
    for a, b in zip(robot, robot[1:]):
        i = bisect.bisect_right(ts, a[4]) - 1
        if i >= 0 and cmd[i][1] < 0.01 and a[3] > 0.05:
            dur += b[0] - a[0]
            dist += math.hypot(b[1] - a[1], b[2] - a[2])
    return {'pushed_s': round(dur, 1), 'pushed_m': round(dist, 2)}


def stalls(robot, t_from):
    out, start = [], None
    for t, x, y, v, *_ in robot:
        if t < t_from:
            continue
        if v < STOP_V:
            start = start or (t, x, y)
        else:
            if start and t - start[0] >= STOP_S:
                out.append((start[0], t, start[1], start[2]))
            start = None
    if start and robot and robot[-1][0] - start[0] >= STOP_S:
        out.append((start[0], robot[-1][0], start[1], start[2]))
    return out


def guest_at(track, t):
    i = min(range(len(track)), key=lambda k: abs(track[k][0] - t))
    return track[i]


def stall_detail(stall, guests, dmap):
    t0, t1, x, y = stall
    near = []
    for gid, tr in guests.items():
        ds, moving = [], []
        for t in np.linspace(t0, t1, 20):
            _, gx, gy, gv = guest_at(tr, t)
            ds.append(math.hypot(gx - x, gy - y))
            moving.append(gv > 0.1)
        near.append((min(ds), float(np.median(ds)), sum(moving) / len(moving), gid))
    near.sort()
    d, med, mov, gid = near[0] if near else (float('inf'), float('inf'), 0.0, '')
    # 손님 몸(0.3)과 로봇 외접(0.39) 사이가 1.0 m 안이면 손님이 막는 것으로 본다
    blocking = [g for g in near if g[0] - GUEST_R - CIRCUMSCRIBED < 1.0]
    return {'t0': round(t0, 1), 'dur_s': round(t1 - t0, 1), 'at': [round(x, 2), round(y, 2)],
            'static_clearance_m': round(clearance(dmap, x, y), 2),
            'nearest_guest': gid, 'nearest_guest_min_m': round(d, 2), 'nearest_guest_median_m': round(med, 2),
            'nearest_guest_moving_frac': round(mov, 2),
            'guests_within_1m_gap': [(g[3], round(g[0], 2), round(g[2], 2)) for g in blocking]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('raw', nargs='+')
    ap.add_argument('--map', required=True)
    ap.add_argument('--out')
    a = ap.parse_args()
    dmap = load_map(a.map)
    rows = []
    for raw in map(Path, a.raw):
        for js in sorted(raw.glob('*.json')):
            label = js.stem
            d = json.loads(js.read_text())
            if not isinstance(d, dict) or not ('traj' in d or 'events' in d):
                continue
            status = d.get('status') or ('done' if d.get('assign_order') else '?')
            row = {'run': f'{raw.name}/{label}', 'status': status, **log_counts(raw / 'logs' / f'{label}.system.log')}
            if 'end' in d and d['end']:
                row['end'] = [round(v, 2) for v in d['end']]
                row['end_static_clearance_m'] = round(clearance(dmap, *d['end']), 2)
            bag = raw / 'bags' / label
            if bag.exists():
                robot, guests, cmd = read_bag(bag)
                if robot:
                    row['pushed'] = pushed(robot, cmd)
                    row['end_gt'] = [round(robot[-1][1], 2), round(robot[-1][2], 2)]
                    row['end_gt_static_clearance_m'] = round(clearance(dmap, robot[-1][1], robot[-1][2]), 2)
                    row['min_gt_static_clearance_m'] = round(min(clearance(dmap, r[1], r[2]) for r in robot), 2)
                    st = stalls(robot, robot[0][0] + 2.0)
                    row['stalls'] = [stall_detail(s, guests, dmap) for s in st]
            rows.append(row)
    for r in rows:
        print(json.dumps(r, ensure_ascii=False))
    if a.out:
        Path(a.out).write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding='utf-8')
    print(f'footprint 0.565 x 0.537: 내접 {INSCRIBED:.3f} m, 외접 {CIRCUMSCRIBED:.3f} m')


if __name__ == '__main__':
    main()
