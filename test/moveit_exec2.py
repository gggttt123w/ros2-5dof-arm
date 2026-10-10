#!/usr/bin/env python3
"""MoveIt2 三维坐标 → 规划 → 执行（修正版）

上一版的 bug：`start` 只在开头读了一次，但执行后机械臂已经移位，
             下一条轨迹的起点就是错的 → JTC 报 PATH_TOLERANCE_VIOLATED。
本版：每次规划前都重读 /joint_states。

另外：机械臂完全伸直时（r≈0）是奇异位形，附近的目标规划/执行都不稳，
     所以先走到一个「弯曲的 ready 姿态」，再在它附近扫描目标。

    python3 moveit_exec2.py            扫描约 8 个目标
    python3 moveit_exec2.py dry        只规划不执行
    python3 moveit_exec2.py sweep      在 ready 姿态周围更密的扫描
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.srv import GetPositionFK, GetMotionPlan
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
    RobotState, MoveItErrorCodes, Constraints, JointConstraint,
    MotionPlanRequest, PositionConstraint, BoundingVolume,
)

GROUP, TIP, BASE = 'arm', 'link5', 'base_link'
JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']

# 远离奇异的 ready 姿态（弯曲的臂）
READY = [0.0, 0.45, -0.70, 0.35, 0.0]

# 在 READY 附近的关节偏移，每个做一次 FK 得到三维坐标
DELTAS = [
    ('j1 +0.20',       [0.20, 0.00,  0.00,  0.00, 0.0]),
    ('j1 -0.20',       [-0.20, 0.00, 0.00,  0.00, 0.0]),
    ('j2 +0.15',       [0.00, 0.15,  0.00,  0.00, 0.0]),
    ('j2 -0.15',       [0.00, -0.15, 0.00,  0.00, 0.0]),
    ('j3 +0.15',       [0.00, 0.00,  0.15,  0.00, 0.0]),
    ('j3 -0.15',       [0.00, 0.00, -0.15,  0.00, 0.0]),
    ('j4 +0.20',       [0.00, 0.00,  0.00,  0.20, 0.0]),
    ('j1+j2 复合',      [0.15, 0.10,  0.00,  0.00, 0.0]),
    ('j2+j3 复合',      [0.00, 0.12, -0.12,  0.00, 0.0]),
    ('大复合',          [0.18, 0.15, -0.20,  0.15, 0.0]),
]

SWEEP = [
    ('%s j1 %+0.2f' % ('', d), [d, 0.0, 0.0, 0.0, 0.0])
    for d in (-0.30, -0.20, -0.10, 0.10, 0.20, 0.30)
] + [
    ('j2+j3 %+0.2f' % d, [0.0, d, -d, 0.0, 0.0])
    for d in (-0.20, -0.12, 0.12, 0.20)
]


def rstate(q):
    rs = RobotState()
    rs.joint_state.name = JOINTS
    rs.joint_state.position = list(q)
    return rs


def call(node, cli, req, t=30.0):
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=t)
    return fut.result()


def read_js(node, timeout=5.0):
    box = {'m': None}

    def cb(m):
        box['m'] = m

    sub = node.create_subscription(JointState, '/joint_states', cb, 10)
    t0 = time.time()
    while box['m'] is None and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.3)
    node.destroy_subscription(sub)
    if box['m'] is None:
        return None
    d = dict(zip(box['m'].name, box['m'].position))
    return [d.get(j, 0.0) for j in JOINTS]


def fk(node, cli, q):
    r = GetPositionFK.Request()
    r.header.frame_id = BASE
    r.fk_link_names = [TIP]
    r.robot_state = rstate(q)
    res = call(node, cli, r, 10.0)
    if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
        return None
    return res.pose_stamped[0].pose


def pos_c(xyz, tol):
    pc = PositionConstraint()
    pc.header.frame_id = BASE
    pc.link_name = TIP
    pc.weight = 1.0
    s = SolidPrimitive()
    s.type = SolidPrimitive.SPHERE
    s.dimensions = [tol]
    p = Pose()
    p.position.x, p.position.y, p.position.z = xyz
    p.orientation.w = 1.0
    bv = BoundingVolume()
    bv.primitives = [s]
    bv.primitive_poses = [p]
    pc.constraint_region = bv
    c = Constraints()
    c.position_constraints = [pc]
    return c


def joint_c(target):
    c = Constraints()
    for name, val in zip(JOINTS, target):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = val
        jc.tolerance_above = 0.02
        jc.tolerance_below = 0.02
        jc.weight = 1.0
        c.joint_constraints.append(jc)
    return c


def plan(node, cli, start, cons, t=6.0):
    req = GetMotionPlan.Request()
    mr = MotionPlanRequest()
    mr.group_name = GROUP
    mr.num_planning_attempts = 5
    mr.allowed_planning_time = t
    mr.start_state = rstate(start)
    # ★ 降速跑：舵机 + 卫星式轮询反馈跟不紧高动态轨迹
    mr.max_velocity_scaling_factor = 0.2
    mr.max_acceleration_scaling_factor = 0.2
    mr.goal_constraints = [cons]
    req.motion_plan_request = mr
    res = call(node, cli, req, t + 25)
    if res is None:
        return None, 'timeout'
    code = res.motion_plan_response.error_code.val
    if code != MoveItErrorCodes.SUCCESS:
        return None, 'code=%d' % code
    return res.motion_plan_response.trajectory, None


def execute(node, cli, traj, t=30.0):
    goal = ExecuteTrajectory.Goal()
    goal.trajectory = traj
    fut = cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=t)
    gh = fut.result()
    if gh is None or not gh.accepted:
        return None
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=t)
    r = rf.result()
    return r.result.error_code.val if r else None


def settle(node, want, tol=0.05, timeout=8.0):
    """等机械臂走完，返回稳定后的关节角"""
    t0 = time.time()
    q = None
    while time.time() - t0 < timeout:
        q = read_js(node)
        if q and all(abs(a - b) < tol for a, b in zip(q, want)):
            return q
        time.sleep(0.4)
    return q


def main():
    argv = sys.argv[1:]
    dry = 'dry' in argv
    cases = SWEEP if 'sweep' in argv else DELTAS

    rclpy.init()
    node = Node('moveit_exec2')
    fk_cli = node.create_client(GetPositionFK, '/compute_fk')
    mp_cli = node.create_client(GetMotionPlan, '/plan_kinematic_path')
    for n, c in (('/compute_fk', fk_cli), ('/plan_kinematic_path', mp_cli)):
        if not c.wait_for_service(timeout_sec=15.0):
            print('FAIL %s 不可用' % n)
            return 1
    ex_cli = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
    if not dry and not ex_cli.wait_for_server(timeout_sec=15.0):
        print('FAIL 找不到 /execute_trajectory')
        return 1

    # ── ① 先走到 ready 姿态（远离奇异）──
    cur = read_js(node)
    if cur is None:
        print('FAIL 读不到 /joint_states')
        return 1
    p = fk(node, fk_cli, cur)
    print('起始关节角: %s' % (['%+.3f' % v for v in cur],))
    if p:
        print('起始末端:   x=%+.4f y=%+.4f z=%+.4f' % (p.position.x, p.position.y, p.position.z))

    if not dry:
        print('')
        print('→ 先走到 ready 姿态 %s（远离奇异）' % (['%+.2f' % v for v in READY],))
        traj, e = plan(node, mp_cli, cur, joint_c(READY))
        if traj is None:
            print('  FAIL 规划: %s' % e)
            return 1
        code = execute(node, ex_cli, traj)
        print('  执行 code=%s' % code)
        if code != MoveItErrorCodes.SUCCESS:
            return 1
        cur = settle(node, READY)
        p = fk(node, fk_cli, cur)
        print('  ready 末端: x=%+.4f y=%+.4f z=%+.4f'
              % (p.position.x, p.position.y, p.position.z))

    print('')
    hdr = '%-18s %-24s %-7s %-7s %s' % ('目标', '三维坐标', '规划', '执行', '实测误差')
    print(hdr)
    print('-' * len(hdr))

    n_ok = n_plan = 0
    errs = []

    for label, delta in cases:
        # ★ 每次都用【最新的】当前状态做起点
        start = read_js(node)
        if start is None:
            print('%-18s 读不到 /joint_states' % label)
            continue

        q_ref = [s + d for s, d in zip(start, delta)]
        ref = fk(node, fk_cli, q_ref)
        if ref is None:
            print('%-18s FK 失败' % label)
            continue
        xyz = (ref.position.x, ref.position.y, ref.position.z)

        traj, e = plan(node, mp_cli, start, pos_c(xyz, 0.010))
        if traj is None:
            print('%-18s (%+.3f,%+.3f,%+.3f)  FAIL %s'
                  % (label, xyz[0], xyz[1], xyz[2], e))
            continue
        n_plan += 1

        if dry:
            print('%-18s (%+.3f,%+.3f,%+.3f)  OK     -       -'
                  % (label, xyz[0], xyz[1], xyz[2]))
            continue

        code = execute(node, ex_cli, traj)
        if code != MoveItErrorCodes.SUCCESS:
            print('%-18s (%+.3f,%+.3f,%+.3f)  OK     FAIL %s'
                  % (label, xyz[0], xyz[1], xyz[2], code))
            time.sleep(1.0)
            continue

        # ★ 舵机走完后还要稳一下；read() 的轮询周期是 240ms，
        #   太早读会读到「还在路上」的值，误差虚高。
        time.sleep(3.0)
        now = read_js(node)
        pe = fk(node, fk_cli, now) if now else None
        if pe is None:
            print('%-18s (%+.3f,%+.3f,%+.3f)  OK     OK      读不到'
                  % (label, xyz[0], xyz[1], xyz[2]))
            n_ok += 1
            continue

        d = math.sqrt((pe.position.x - xyz[0]) ** 2 +
                      (pe.position.y - xyz[1]) ** 2 +
                      (pe.position.z - xyz[2]) ** 2)
        errs.append(d)
        n_ok += 1
        print('%-18s (%+.3f,%+.3f,%+.3f)  OK     OK      %5.1f mm  %s'
              % (label, xyz[0], xyz[1], xyz[2], d * 1000,
                 'OK' if d < 0.010 else 'OUT'))

    print('')
    print('规划 %d/%d   执行 %d/%d' % (n_plan, len(cases), n_ok, len(cases)))
    if errs:
        print('末端误差: 平均 %.1f mm  最大 %.1f mm' % (
            sum(errs) / len(errs) * 1000, max(errs) * 1000))

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
