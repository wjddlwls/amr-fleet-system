"""시연 2 (회수 호출): 일반 작업 몇 개를 넣고 회수 호출(TYPE_COLLECT)을 넣어 할당 순서·경로를 기록한다.

    python3 scene2.py --stations <stations_restaurant.yaml> --out /tmp/r2/<label>.json

created_time은 /clock(시뮬레이션 시간)으로 채운다 - aging이 대기 시간을 서버 시계로 재기 때문이다(api.md 4-6).
"""
import argparse
import json
import math
import time

import rclpy
import yaml
from amr_msgs.msg import RobotState, Task, TaskAssignment, TaskStatus
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

# (보낼 시각 s, task_id, 종류, 픽업, 하역, priority)
SCHEDULE = [
    (0.0, 'n1_stall1_t03', Task.TYPE_DELIVERY, 'stall_1', 't03', 100),
    (0.0, 'n2_stall2_t08', Task.TYPE_DELIVERY, 'stall_2', 't08', 100),
    (1.0, 'n3_stall3_t09', Task.TYPE_DELIVERY, 'stall_3', 't09', 100),
    (1.0, 'n4_stall4_t14', Task.TYPE_DELIVERY, 'stall_4', 't14', 100),
    (5.0, 'c1_t07_return1', Task.TYPE_COLLECT, 't07', 'return_1', 200),
    (10.0, 'c2_t13_return2', Task.TYPE_COLLECT, 't13', 'return_2', 200),
]
STATE = {0: 'PENDING', 1: 'IN_PROGRESS', 2: 'COMPLETED', 3: 'FAILED'}


def pose_msg(p):
    from geometry_msgs.msg import Pose
    m = Pose()
    m.position.x, m.position.y = float(p[0]), float(p[1])
    m.orientation.z, m.orientation.w = math.sin(p[2] / 2), math.cos(p[2] / 2)
    return m


class Scene(Node):
    def __init__(self, stations, robots):
        super().__init__('scene2', parameter_overrides=[rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.st = stations
        self.robots = robots
        self.tf = Buffer()
        TransformListener(self.tf, self)
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Task, '/fleet/task_requests', rel)
        self.events = []
        self.states = {}
        self.create_subscription(TaskAssignment, '/fleet/task_assignments', self.on_assign, rel)
        self.create_subscription(TaskStatus, '/fleet/task_status', self.on_status, rel)
        self.create_subscription(RobotState, '/fleet/robot_states', lambda m: self.states.__setitem__(m.robot_id, m),
                                 10)
        self.seen_assign = set()
        self.last_status = {}
        self.t0 = None

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def rel(self):
        return round(self.now() - self.t0, 2) if self.t0 is not None else None

    def where(self, robot):
        try:
            t = self.tf.lookup_transform('map', f'{robot}/base_footprint', Time())
            return [round(t.transform.translation.x, 2), round(t.transform.translation.y, 2)]
        except Exception:
            return None

    def on_assign(self, m):
        key = (m.assignment_id, m.robot_id)
        if key in self.seen_assign:
            return
        self.seen_assign.add(key)
        self.events.append({'t': self.rel(), 'ev': 'ASSIGN' if m.action == 0 else 'CANCEL', 'robot': m.robot_id,
                            'task': m.task.task_id, 'at': self.where(m.robot_id)})

    def on_status(self, m):
        key = (m.state, m.phase)
        if self.last_status.get(m.task_id) == key:
            return
        self.last_status[m.task_id] = key
        self.events.append({'t': self.rel(), 'ev': STATE.get(m.state, str(m.state)), 'phase': m.phase,
                            'robot': m.robot_id, 'task': m.task_id, 'at': self.where(m.robot_id),
                            'err': m.error.code})

    def send(self, task_id, ttype, pickup, dropoff, priority):
        t = Task()
        t.task_id, t.task_type, t.item_type = task_id, ttype, Task.ITEM_NONE
        t.pickup_station_id, t.pickup_pose = pickup, pose_msg(self.st[pickup])
        t.dropoff_station_id, t.dropoff_pose = dropoff, pose_msg(self.st[dropoff])
        t.priority = priority
        t.created_time = self.get_clock().now().to_msg()
        self.pub.publish(t)
        self.events.append({'t': self.rel(), 'ev': 'REQUEST', 'task': task_id, 'priority': priority})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--stations', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--robots', nargs='+', default=['amr_01', 'amr_02'])
    ap.add_argument('--timeout', type=float, default=900.0, help='시뮬레이션 s')
    a = ap.parse_args()
    stations = yaml.safe_load(open(a.stations))['stations']
    rclpy.init()
    n = Scene(stations, a.robots)
    w0 = time.time()
    while not all(r in n.states for r in a.robots) or any(n.where(r) is None for r in a.robots) \
            or n.count_subscribers('/fleet/task_requests') == 0:
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.time() - w0 > 900:
            raise SystemExit('로봇 상태/TF/서버 준비 안 됨')
    end = time.time() + 5.0
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    n.t0 = n.now()
    wall0 = time.time()
    start_pose = {r: n.where(r) for r in a.robots}
    pending = list(SCHEDULE)
    done = set()
    ids = {s[1] for s in SCHEDULE}
    stopped = 'done'
    try:
        while True:
            rclpy.spin_once(n, timeout_sec=0.02)
            while pending and n.now() - n.t0 >= pending[0][0]:
                _, *spec = pending.pop(0)
                n.send(*spec)
            done = {e['task'] for e in n.events if e['ev'] in ('COMPLETED', 'FAILED')}
            if ids <= done:
                break
            if n.now() - n.t0 > a.timeout:
                stopped = 'timeout'
                break
    except KeyboardInterrupt:
        stopped = 'interrupted'
    sim_s, wall_s = n.now() - n.t0, time.time() - wall0
    order = [e['task'] for e in n.events if e['ev'] == 'ASSIGN']
    out = {'stopped': stopped, 'start_pose': start_pose, 'sim_s': round(sim_s, 1), 'wall_s': round(wall_s, 1),
           'rtf': round(sim_s / wall_s, 3), 'assign_order': order, 'done': sorted(done), 'events': n.events}
    json.dump(out, open(a.out, 'w'), indent=1)
    print(json.dumps({k: out[k] for k in ('stopped', 'sim_s', 'wall_s', 'rtf', 'assign_order', 'done')}))
    for e in n.events:
        print(e)
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
