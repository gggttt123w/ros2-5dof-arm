#!/usr/bin/env python3
"""MoveIt2 端到端执行测试：三维坐标 → 规划 → ★真正驱动机械臂★ → 实测误差

    python3 moveit_exec.py            跑默认的多个目标（小幅度，安全）
    python3 moveit_exec.py dry        只规划不执行
    python3 moveit_exec.py span 0.25  用更大的关节偏移生成目标

流程：
    ① 读当前 /joint_states
    ② 用「当前关节角 + 小偏移」做 FK，得到【一定可达】的三维坐标
    ③ 用位置约束（球形容差）调 /plan_kinematic_path 规划
    ④ 调 /execute_trajectory 真正执行
    ⑤ 等稳定后读回 /joint_states，用 FK 算出实际末端位置，报误差
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
    RobotState, MoveItErrorCodes, Constraints, MotionPlanRequest,
    PositionConstraint, BoundingVolume, RobotTrajectory,
)

GROUP, TIP, BASE = 'arm', 'link5', 'base_link'
JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']

# 默认目标：每个只动一个关节一点，安全
DEFAULT_DELTAS = [
    ('joint1 +0.15', [0.15, 0.0, 0.0, 0.0, 0.0]),
    ('joint1 -0.15', [-0.15, 0.0, 0.0, 0.0, 0.0]),
    ('joint2 +0.12', [0.0, 0.12, 0.0, 0.0, 0.0]),
    ('joint3 -0.12', [0.0, 0.0, -0.12, 0.0, 0.0]),
    ('joint2+3 复合', [0.10, 0.10, -0.10, 0.0, 0.0]),
    ('joint1+2+4', [0.12, 0.08, 0.0, 0.10, 0.0]),
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


def read_js(node, timeout=6.0):
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


def plan(node, cli, start, cons, t=6.0):
    req = GetMotionPlan.Request()
    mr = MotionPlanRequest()
    mr.group_name = GROUP
    mr.num_planning_attempts = 5
    mr.allowed_planning_time = t
    mr.start_state = rstate(start)
    mr.max_velocity_scaling_factor = 0.3
    mr.max_acceleration_scaling_factor = 0.3
    mr.goal_constraints = [cons]
    req.motion_plan_request = mr
    res = call(node, cli, req, t + 25)
    if res is None:
        return None, 'timeout'
    if res.motion_plan_response.error_code.val != MoveItErrorCodes.SUCCESS:
        return None, 'code=%d' % res.motion_plan_response.error_code.val
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


def main():
    argv = sys.argv[1:]
    dry = 'dry' in argv
    span = 1.0
    if 'span' in argv:
        span = float(argv[argv.index('span') + 1])

    rclpy.init()
    node = Node('moveit_exec')

    fk_cli = node.create_client(GetPositionFK, '/compute_fk')
    mp_cli = node.create_client(GetMotionPlan, '/plan_kinematic_path')
    for n, c in (('/compute_fk', fk_cli), ('/plan_kinematic_path', mp_cli)):
        if not c.wait_for_service(timeout_sec=15.0):
            print('FAIL %s 不可用' % n)
            return 1

    ex_cli = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')

    start = read_js(node)
    if start is None:
        print('FAIL 读不到 /joint_states')
        return 1
    print('当前关节角: %s' % (['%+.4f' % v for v in start],))
    p0 = fk(node, fk_cli, start)
    if p0:
        print('当前末端位置: x=%+.4f y=%+.4f z=%+.4f'
              % (p0.position.x, p0.position.y, p0.position.z))
    print('模式: %s   关节偏移缩放: %.2f' % ('只规划' if dry else '规划+执行', span))
    print('')

    hdr = ('%-16s %-24s %-8s %-10s %s'
           % ('目标', '三维坐标', '规划', '执行', '实测误差'))
    print(hdr)
    print('-' * len(hdr))

    n_ok = n_plan = n_total = 0
    errs = []

    for label, delta in DEFAULT_DELTAS:
        n_total += 1
        q_ref = [s + d * span for s, d in zip(start, delta)]
        ref = fk(node, fk_cli, q_ref)
        if ref is None:
            print('%-16s FK 失败' % label)
            continue
        xyz = (ref.position.x, ref.position.y, ref.position.z)

        traj, e = plan(node, mp_cli, start, pos_c(xyz, 0.010))
        if traj is None:
            print('%-16s (%+.3f,%+.3f,%+.3f)   FAIL %s'
                  % (label, xyz[0], xyz[1], xyz[2], e))
            continue
        n_plan += 1

        if dry:
            print('%-16s (%+.3f,%+.3f,%+.3f)   OK      -          -'
                  % (label, xyz[0], xyz[1], xyz[2]))
            continue

        if not ex_cli.wait_for_server(timeout_sec=10.0):
            print('FAIL 找不到 /execute_trajectory')
            return 1

        code = execute(node, ex_cli, traj)
        if code != MoveItErrorCodes.SUCCESS:
            print('%-16s (%+.3f,%+.3f,%+.3f)   OK      FAIL code=%s'
                  % (label, xyz[0], xyz[1], xyz[2], code))
            continue
        n_ok += 1

        # 等机械臂走完 + 反馈刷新
        time.sleep(2.5)
        now = read_js(node)
        pe = fk(node, fk_cli, now) if now else None
        if pe is None:
            print('%-16s (%+.3f,%+.3f,%+.3f)   OK      OK         读不到'
                  % (label, xyz[0], xyz[1], xyz[2]))
            continue

        d = math.sqrt((pe.position.x - xyz[0]) ** 2 +
                      (pe.position.y - xyz[1]) ** 2 +
                      (pe.position.z - xyz[2]) ** 2)
        errs.append(d)
        print('%-16s (%+.3f,%+.3f,%+.3f)   OK      OK        %5.1f mm  %s'
              % (label, xyz[0], xyz[1], xyz[2], d * 1000,
                 'OK' if d < 0.010 else 'OUT'))

    print('')
    print('规划成功 %d/%d，执行成功 %d/%d' % (n_plan, n_total, n_ok, n_total))
    if errs:
        print('末端定位误差: 平均 %.1f mm，最大 %.1f mm'
              % (sum(errs) / len(errs) * 1000, max(errs) * 1000))

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
