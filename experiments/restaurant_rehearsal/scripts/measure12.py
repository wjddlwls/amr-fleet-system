"""시연 1-2 측정: amr_01에 목표 하나를 보내고 경로·주행 시간·구역 통과·구역 안 속도·RTF를 잰다.

    python3 measure12.py --goal -9.9 4.15 1.5708 --out /tmp/r12/<label>.json
"""
import argparse
import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from amr_msgs.msg import ZoneCongestionArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

ZONES = {'west_aisle': (-11.9, -9.73, -5.0, 2.99), 'buffet_line': (-11.9, -6.1, 3.0, 4.63),
         'hall_west': (-9.72, -1.2, -5.0, 2.99)}


def inside(x, y, z):
    a = ZONES[z]
    return a[0] <= x <= a[1] and a[2] <= y <= a[3]


def plen(pts):
    return sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def zone_len(pts, z):
    return sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1)
               if inside(*pts[i], z) and inside(*pts[i + 1], z))


class M(Node):
    def __init__(self, ns):
        super().__init__('measure12', parameter_overrides=[rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.ns = ns
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.ac = ActionClient(self, NavigateToPose, f'/{ns}/navigate_to_pose')
        self.plans = []
        self.congestion = {}
        self.mask = None
        self.create_subscription(Path, f'/{ns}/plan', self.on_plan, 10)
        self.create_subscription(ZoneCongestionArray, '/fleet/congestion', self.on_cong, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/fleet/congestion_mask', lambda m: setattr(self, 'mask', m), latched)

    def on_plan(self, msg):
        if self.recording:
            self.plans.append([(p.pose.position.x, p.pose.position.y) for p in msg.poses])

    def on_cong(self, msg):
        for z in msg.zones:
            self.congestion[z.zone_id] = round(float(z.value), 2)

    recording = False

    def pose(self):
        try:
            t = self.tf.lookup_transform('map', f'{self.ns}/base_footprint', Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None


def spin_for(node, sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ns', default='amr_01')
    ap.add_argument('--goal', nargs=3, type=float, required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--timeout', type=float, default=240.0)   # 시뮬레이션 s
    ap.add_argument('--need-mask', action='store_true')
    a = ap.parse_args()
    rclpy.init()
    n = M(a.ns)
    t0 = time.time()
    while not n.ac.wait_for_server(timeout_sec=1.0) or n.pose() is None or (a.need_mask and n.mask is None):
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.time() - t0 > 600:
            raise SystemExit('준비 안 됨 (action server / TF / mask)')
    spin_for(n, 10.0)   # 혼잡도가 자리잡게 (EMA alpha 0.5, 2 Hz)
    cong_start = dict(n.congestion)
    g = NavigateToPose.Goal()
    g.pose = PoseStamped()
    g.pose.header.frame_id = 'map'
    g.pose.pose.position.x, g.pose.pose.position.y = a.goal[0], a.goal[1]
    g.pose.pose.orientation.z = math.sin(a.goal[2] / 2)
    g.pose.pose.orientation.w = math.cos(a.goal[2] / 2)
    n.recording = True
    start_pose = n.pose()
    sim0, wall0 = n.get_clock().now(), time.time()
    fut = n.ac.send_goal_async(g)
    rclpy.spin_until_future_complete(n, fut)
    gh = fut.result()
    res = gh.get_result_async()
    traj = []   # (sim_s, x, y)
    cong_max = {}
    last = 0.0
    while not res.done():
        rclpy.spin_once(n, timeout_sec=0.02)
        now = (n.get_clock().now() - sim0).nanoseconds * 1e-9
        if now - last >= 0.1:
            p = n.pose()
            if p:
                traj.append((now, *p))
            last = now
            for k, v in n.congestion.items():
                cong_max[k] = max(cong_max.get(k, 0.0), v)
        if now > a.timeout:
            gh.cancel_goal_async()
            spin_for(n, 2.0)
            break
    sim_s = (n.get_clock().now() - sim0).nanoseconds * 1e-9
    wall_s = time.time() - wall0
    status = res.result().status if res.done() else -1
    pts = [(x, y) for _, x, y in traj]
    # 구역 안 속도: 0.5 s 창
    speeds_in = []
    for i in range(5, len(traj)):
        t1, x1, y1 = traj[i]
        t0_, x0, y0 = traj[i - 5]
        if t1 > t0_ and inside(x1, y1, 'west_aisle') and inside(x0, y0, 'west_aisle'):
            speeds_in.append(math.hypot(x1 - x0, y1 - y0) / (t1 - t0_))
    speeds_all = [math.hypot(traj[i][1] - traj[i - 5][1], traj[i][2] - traj[i - 5][2]) / (traj[i][0] - traj[i - 5][0])
                  for i in range(5, len(traj)) if traj[i][0] > traj[i - 5][0]]
    first_plan = n.plans[0] if n.plans else []
    # 배식대 앞 통로 직전(y 2.3, 테이블 1행 북쪽 끝)에 처음 닿은 시각과 그때 x (서쪽 통로 x < -9.73 / 돌아간 통로)
    reach = next(((t, x) for t, x, y in traj if y >= 2.3), None)
    out = {
        'status': {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
                   GoalStatus.STATUS_CANCELED: 'CANCELED'}.get(status, str(status)),
        'start': start_pose, 'goal': a.goal, 'end': pts[-1] if pts else None,
        'sim_s': round(sim_s, 2), 'wall_s': round(wall_s, 2), 'rtf': round(sim_s / wall_s, 3),
        'driven_m': round(plen(pts), 2), 'driven_in_west_aisle_m': round(zone_len(pts, 'west_aisle'), 2),
        'reach_y23_s': round(reach[0], 2) if reach else None,
        'reach_y23_x': round(reach[1], 2) if reach else None,
        'goal_dist_end': round(math.dist(pts[-1], a.goal[:2]), 2) if pts else None,
        'first_plan_m': round(plen(first_plan), 2),
        'first_plan_in_west_aisle_m': round(zone_len(first_plan, 'west_aisle'), 2),
        'plans': len(n.plans),
        'plans_through_west_aisle': sum(zone_len(p, 'west_aisle') > 0.5 for p in n.plans),
        'speed_in_west_aisle_mean': round(sum(speeds_in) / len(speeds_in), 3) if speeds_in else None,
        'speed_in_west_aisle_max': round(max(speeds_in), 3) if speeds_in else None,
        'speed_mean_all': round(sum(speeds_all) / len(speeds_all), 3) if speeds_all else None,
        'speed_max_all': round(max(speeds_all), 3) if speeds_all else None,
        'congestion_at_start': cong_start, 'congestion_max': cong_max,
        'traj': [(round(t, 2), round(x, 3), round(y, 3)) for t, x, y in traj],
        'first_plan': [(round(x, 3), round(y, 3)) for x, y in first_plan[::5]],
    }
    with open(a.out, 'w') as f:
        json.dump(out, f)
    print(json.dumps({k: v for k, v in out.items() if k not in ('traj', 'first_plan')}))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
