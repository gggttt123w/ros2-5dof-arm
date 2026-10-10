#!/usr/bin/env python3
"""大幅运动测试：三维坐标 → MoveIt2 规划 → 执行（大幅度）

    python3 moveit_big.py            执行大幅序列
    python3 moveit_big.py dry        只规划不执行（先看能不能规划）

安全设计：
  · 所有目标都由 FK 从合法关节角生成 → 保证【可达】
  · 速度缩放 0.3，慢走
  · 每个目标单独判成败，失败就跳过，不中断
  · 每个目标移动后等机械臂停稳再读回，报实测误差
  · 开头先走到 ready（远离奇异）
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
READY = [0.0, 0.45, -0.70, 0.35, 0.0]

# 大幅度目标（关节角都在限位内：joint1~4 ±1.5708，joint5 ±3.1416）
CONFIGS = [
    ('① 前伸水平',    [0.00, 1.45,  0.00,  0.00,  0.0]),
    ('② 前伸下压',    [0.00, 1.20, -0.60,  0.30,  0.0]),
    ('③ 左侧大幅',    [1.30, 1.00, -0.40,  0.20,  0.0]),
    ('④ 右侧大幅',    [-1.30, 1.00, -0.40, 0.20,  0.0]),
    ('⑤ 后收上举',    [0.00, -0.60, 1.20, -0.50,  0.0]),
    ('⑥ 复合+腕自转',  [1.20, 0.90, -1.10, 0.60,  2.5]),
    ('⑦ 复合-腕自转',  [-1.20, 0.90, -1.10, 0.60, -2.5]),
    ('⑧ 回归 ready',  [0.00, 0.45, -0.70, 0.35,  0.0]),
]

TOL = 0.020          # 三维坐标容差 20mm（大幅运动放宽）
SCALE = 0.3          # 速度缩放


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


def plan(node, cli, start, cons, t=8.0):
    req = GetMotionPlan.Request()
    mr = MotionPlanRequest()
    mr.group_name = GROUP
    mr.num_planning_attempts = 5
    mr.allowed_planning_time = t
    mr.start_state = rstate(start)
    mr.max_velocity_scaling_factor = SCALE
    mr.max_acceleration_scaling_factor = SCALE
    mr.goal_constraints = [cons]
    req.motion_plan_request = mr
    res = call(node, cli, req, t + 30)
    if res is None:
        return None, 'timeout'
    code = res.motion_plan_response.error_code.val
    if code != MoveItErrorCodes.SUCCESS:
        return None, 'code=%d' % code
    return res.motion_plan_response.trajectory, None


def execute(node, cli, traj, t=45.0):
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


def traj_duration(traj):
    # traj 是 RobotTrajectory，点在里面那层 joint_trajectory 里
    pts = traj.joint_trajectory.points
    if not pts:
        return 0.0
    d = pts[-1].time_from_start
    return d.sec + d.nanosec * 1e-9


def main():
    dry = 'dry' in sys.argv[1:]

    rclpy.init()
    node = Node('moveit_big')
    fk_cli = node.create_client(GetPositionFK, '/compute_fk')
    mp_cli = node.create_client(GetMotionPlan, '/plan_kinematic_path')
    for n, c in (('/compute_fk', fk_cli), ('/plan_kinematic_path', mp_cli)):
        if not c.wait_for_service(timeout_sec=15.0):
            print('FAIL %s 不可用 —— move_group 起了吗？' % n)
            return 1
    ex_cli = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
    if not dry and not ex_cli.wait_for_server(timeout_sec=15.0):
        print('FAIL 找不到 /execute_trajectory —— ros2_control 起了吗？')
        return 1

    cur = read_js(node)
    if cur is None:
        print('FAIL 读不到 /joint_states')
        return 1
    p = fk(node, fk_cli, cur)
    print('起始关节角: %s' % (['%+.3f' % v for v in cur],))
    if p:
        print('起始末端:   x=%+.4f y=%+.4f z=%+.4f'
              % (p.position.x, p.position.y, p.position.z))
    print('幅度: joint1 ±1.30  joint2 ~1.45  joint3 ~-1.10  joint5 ±2.50')
    print('容差 %.0f mm，速度缩放 %.1f，%s' % (TOL * 1000, SCALE,
                                             '只规划' if dry else '规划+执行'))
    print('')

    # ── 先走到 ready ──
    if not dry:
        print('→ 先回到 ready 姿态')
        traj, e = plan(node, mp_cli, cur, joint_c(READY))
        if traj is None:
            print('  FAIL 规划: %s' % e)
            return 1
        print('  轨迹时长 %.1fs，执行…' % traj_duration(traj))
        code = execute(node, ex_cli, traj)
        if code != MoveItErrorCodes.SUCCESS:
            print('  FAIL 执行 code=%s' % code)
            return 1
        time.sleep(3.0)
        cur = read_js(node)
        p = fk(node, fk_cli, cur)
        print('  ready 末端: x=%+.4f y=%+.4f z=%+.4f'
              % (p.position.x, p.position.y, p.position.z))
        print('')

    hdr = '%-16s %-26s %-9s %-7s %s' % ('目标', '三维坐标', '规划', '执行', '实测误差')
    print(hdr)
    print('-' * len(hdr))

    n_plan = n_exec = 0
    errs = []

    for label, cfg in CONFIGS:
        start = read_js(node)
        if start is None:
            print('%-16s 读不到 /joint_states' % label)
            continue

        ref = fk(node, fk_cli, cfg)
        if ref is None:
            print('%-16s FK 失败' % label)
            continue
        xyz = (ref.position.x, ref.position.y, ref.position.z)

        traj, e = plan(node, mp_cli, start, pos_c(xyz, TOL))
        if traj is None:
            print('%-16s (%+.3f,%+.3f,%+.3f)  FAIL %s'
                  % (label, xyz[0], xyz[1], xyz[2], e))
            time.sleep(0.5)
            continue
        n_plan += 1
        dur = traj_duration(traj)

        if dry:
            print('%-16s (%+.3f,%+.3f,%+.3f)  OK %.1fs  -       -'
                  % (label, xyz[0], xyz[1], xyz[2], dur))
            continue

        code = execute(node, ex_cli, traj)
        if code != MoveItErrorCodes.SUCCESS:
            print('%-16s (%+.3f,%+.3f,%+.3f)  OK %.1fs  FAIL %s'
                  % (label, xyz[0], xyz[1], xyz[2], dur, code))
            time.sleep(1.0)
            continue
        n_exec += 1

        # 等机械臂真正停稳（轨迹时长 + 余量）
        time.sleep(max(2.0, dur * 0.3 + 2.0))
        now = read_js(node)
        pe = fk(node, fk_cli, now) if now else None
        if pe is None:
            print('%-16s (%+.3f,%+.3f,%+.3f)  OK %.1fs  OK      读不到'
                  % (label, xyz[0], xyz[1], xyz[2], dur))
            continue

        d = math.sqrt((pe.position.x - xyz[0]) ** 2 +
                      (pe.position.y - xyz[1]) ** 2 +
                      (pe.position.z - xyz[2]) ** 2)
        errs.append(d)
        print('%-16s (%+.3f,%+.3f,%+.3f)  OK %.1fs  OK      %5.1f mm  %s'
              % (label, xyz[0], xyz[1], xyz[2], dur, d * 1000,
                 'OK' if d < TOL else 'OUT'))

    print('')
    print('规划 %d/%d   执行 %d/%d' % (n_plan, len(CONFIGS), n_exec, len(CONFIGS)))
    if errs:
        print('末端误差: 平均 %.1f mm  最大 %.1f mm'
              % (sum(errs) / len(errs) * 1000, max(errs) * 1000))

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
