"""스캔-지도 일치: 정답 위치(GT)에서 찍은 스캔을 지도 두 장(경로 계획용 restaurant_gt, 위치추정용 restaurant_scan)에
AMCL likelihood_field와 같은 식으로 맞춰 본다. 컨테이너에서 돈다 (rosbag2_py).

    python3 scan_match.py <bag> --maps <gt.yaml> <scan.yaml> [--every 1.0]

스캔마다 GT 자세를 dx, dy ±0.6 m(0.1 m 간격), dyaw ±10°(5° 간격)로 옮겨 가며 점수를 낸다.
- 점수: 빔 끝점마다 z_hit·exp(-d²/2σ²) + z_rand/max_range 의 평균 (amcl.yaml sigma_hit 0.05, z_hit 0.9, z_rand 0.1)
- peak_offset_m: 점수가 가장 높은 자리가 GT에서 얼마나 떨어졌나 (0이면 지도가 GT를 가장 그럴듯하게 본다)
- flat_frac: GT 점수의 95 % 이상인 후보 비율 (클수록 그 근처 어디든 비슷해 위치를 못 잡는다)
- hit_mean_m: GT 자세에서 빔 끝점과 가장 가까운 점유 칸 사이 평균 거리
- amcl_std_m: amcl_pose 공분산 sqrt(xx + yy) (파티클이 얼마나 퍼졌나)
- slip_mps: 바퀴 오도메트리(/amr_01/odom, 없으면 cmd_vel) 속도 - GT 속도 (양수면 바퀴는 도는데 몸이 덜 간다)
- onset: AMCL 오차가 처음 0.3 m를 넘은 순간의 앞뒤 행
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

SIGMA, Z_HIT, Z_RAND, MAX_RANGE = 0.05, 0.9, 0.1, 20.0
LIDAR_X = 0.15        # amr_description extrinsics lidar xyz (base_link 앞 0.15 m)
MAX_BEAMS = 180


def load_dist(yaml_path):
    m = yaml.safe_load(open(yaml_path))
    img = np.array(Image.open(Path(yaml_path).parent / m['image']))
    dist = ndimage.distance_transform_edt(img >= 128) * m['resolution']   # 점유(검정)까지 거리
    return dist, m['resolution'], m['origin'][:2]


def lookup(dmap, xs, ys):
    dist, res, (ox, oy) = dmap
    h, w = dist.shape
    c = np.clip(((xs - ox) / res).astype(int), 0, w - 1)
    r = np.clip(h - 1 - ((ys - oy) / res).astype(int), 0, h - 1)
    return dist[r, c]


def score(dmap, pts_robot, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    wx = x + c * pts_robot[:, 0] - s * pts_robot[:, 1]
    wy = y + s * pts_robot[:, 0] + c * pts_robot[:, 1]
    d = lookup(dmap, wx, wy)
    return float(np.mean(Z_HIT * np.exp(-d * d / (2 * SIGMA * SIGMA)) + Z_RAND / MAX_RANGE)), float(np.mean(d))


def read(bag):
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    gt, scans, amcl, wheel, cmd = [], [], [], [], []
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    while r.has_next():
        t, d, stamp = r.read_next()
        if t == '/amr_01/ground_truth/odom':
            m = deserialize_message(d, Odometry)
            q = m.pose.pose.orientation
            gt.append((stamp * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y,
                       math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)))
        elif t == '/amr_01/scan':
            scans.append((stamp * 1e-9, deserialize_message(d, LaserScan)))
        elif t == '/amr_01/amcl_pose':
            m = deserialize_message(d, PoseWithCovarianceStamped)
            c = m.pose.covariance
            amcl.append((stamp * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y, math.sqrt(max(0.0, c[0] + c[7]))))
        elif t == '/amr_01/odom':
            m = deserialize_message(d, Odometry)
            wheel.append((stamp * 1e-9, math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)))
        elif t == '/amr_01/cmd_vel':
            m = deserialize_message(d, Twist)
            cmd.append((stamp * 1e-9, abs(m.linear.x)))
    return gt, scans, amcl, wheel, cmd


def gt_speed(gt, t, win=0.5):
    """GT 위치 차분 속도 (t 앞뒤 win s)."""
    a, b = nearest(gt, t - win), nearest(gt, t + win)
    return math.hypot(b[1] - a[1], b[2] - a[2]) / max(1e-3, b[0] - a[0])


def nearest(track, t):
    i = min(range(len(track)), key=lambda k: abs(track[k][0] - t))
    return track[i]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--maps', nargs=2, required=True)
    ap.add_argument('--every', type=float, default=1.0)
    ap.add_argument('--out')
    a = ap.parse_args()
    maps = {Path(p).stem: load_dist(p) for p in a.maps}
    gt, scans, amcl, wheel, cmd = read(a.bag)
    vel = wheel or cmd
    rows, last = [], -1e9
    offsets = [(dx, dy, dyaw) for dx in np.arange(-0.6, 0.61, 0.1) for dy in np.arange(-0.6, 0.61, 0.1)
               for dyaw in np.radians([-10, -5, 0, 5, 10])]
    for t, scan in scans:
        if t - last < a.every:
            continue
        last = t
        _, x, y, yaw = nearest(gt, t)
        ang = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        rng = np.array(scan.ranges)
        ok = np.isfinite(rng) & (rng > scan.range_min) & (rng < min(scan.range_max, MAX_RANGE))
        idx = np.nonzero(ok)[0]
        idx = idx[:: max(1, len(idx) // MAX_BEAMS)]
        pts = np.stack([LIDAR_X + rng[idx] * np.cos(ang[idx]), rng[idx] * np.sin(ang[idx])], axis=1)
        row = {'t': round(t - scans[0][0], 1), 'gt': [round(x, 2), round(y, 2)]}
        if amcl:
            _, ax, ay, astd = nearest(amcl, t)
            row['amcl_err_m'] = round(math.hypot(ax - x, ay - y), 2)
            row['amcl_std_m'] = round(astd, 3)
        if vel:
            row['slip_mps'] = round(nearest(vel, t)[1] - gt_speed(gt, t), 3)
        for name, dmap in maps.items():
            s0, hit = score(dmap, pts, x, y, yaw)
            cand = [(score(dmap, pts, x + dx, y + dy, yaw + dw)[0], dx, dy) for dx, dy, dw in offsets]
            best = max(cand)
            row[name] = {'score_gt': round(s0, 3), 'hit_mean_m': round(hit, 3),
                         'peak_offset_m': round(math.hypot(best[1], best[2]), 2),
                         'flat_frac': round(sum(c[0] >= 0.95 * s0 for c in cand) / len(cand), 3)}
        rows.append(row)
    summary = {}
    for name in maps:
        summary[name] = {k: round(float(np.mean([r[name][k] for r in rows])), 3)
                         for k in ('score_gt', 'hit_mean_m', 'peak_offset_m', 'flat_frac')}
    if any('amcl_err_m' in r for r in rows):
        errs = [r['amcl_err_m'] for r in rows if 'amcl_err_m' in r]
        summary['amcl_err_m'] = {'mean': round(float(np.mean(errs)), 2), 'max': round(max(errs), 2)}
        i = next((k for k, r in enumerate(rows) if r.get('amcl_err_m', 0) > 0.3), None)
        if i is not None:
            summary['onset'] = rows[max(0, i - 2):i + 2]
    summary['slip_source'] = 'odom' if wheel else ('cmd_vel' if cmd else None)
    print(json.dumps({'bag': str(a.bag), 'scans': len(rows), 'summary': summary}, ensure_ascii=False))
    if a.out:
        Path(a.out).write_text(json.dumps({'summary': summary, 'rows': rows}, ensure_ascii=False, indent=1))


if __name__ == '__main__':
    main()
