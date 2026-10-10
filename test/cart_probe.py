#!/usr/bin/env python3
"""找出「三维坐标规划」为什么失败 —— 一次跑 3 种目标约束做对比

    python3 cart_probe.py

变体：
    V0  参考状态合法性（/check_state_validity）
    V1  纯位置约束，容差 10mm
    V2  纯位置约束，容差 50mm     ← 如果这个能过，说明是采样太稀
    V3  位置 + 姿态约束           ← 如果这个能过，说明是姿态采样的问题
"""
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.srv import (
    GetPositionFK, GetPositionIK, GetMotionPlan, GetStateValidity,
)
from moveit_msgs.msg import (
    RobotState, MoveItErrorCodes, Constraints, MotionPlanRequest,
    PositionConstraint, OrientationConstraint, BoundingVolume,
)

GROUP, TIP, BASE = 'arm', 'link5', 'base_link'
JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
Q_REF = [0.4, 0.6, -0.8, 0.3, 0.2]

ERR = {v: k for k, v in vars(MoveItErrorCodes).items()
       if isinstance(v, int) and not k.startswith('_')}


def ename(c):
    return ERR.get(c, 'code=%s' % c)


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
    from sensor_msgs.msg import JointState
    box = {'m': None}

    def cb(m):
        box['m'] = m

    node.create_subscription(JointState, '/joint_states', cb, 10)
    t0 = time.time()
    while box['m'] is None and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.3)
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


def ori_c(qpose, tol):
    oc = OrientationConstraint()
    oc.header.frame_id = BASE
    oc.link_name = TIP
    oc.orientation = qpose.orientation
    oc.absolute_x_axis_tolerance = tol
    oc.absolute_y_axis_tolerance = tol
    oc.absolute_z_axis_tolerance = tol
    oc.weight = 1.0
    return oc


def tryplan(node, cli, start, cons, label, t=8.0):
    req = GetMotionPlan.Request()
    mr = MotionPlanRequest()
    mr.group_name = GROUP
    mr.num_planning_attempts = 3
    mr.allowed_planning_time = t
    mr.start_state = rstate(start)
    mr.max_velocity_scaling_factor = 0.3
    mr.max_acceleration_scaling_factor = 0.3
    mr.goal_constraints = [cons]
    req.motion_plan_request = mr
    res = call(node, cli, req, t + 20)
    if res is None:
        print('   %-34s 超时' % label)
        return False
    code = res.motion_plan_response.error_code.val
    if code != MoveItErrorCodes.SUCCESS:
        print('   %-34s ❌ %s' % (label, ename(code)))
        return False
    tr = res.motion_plan_response.trajectory.joint_trajectory
    dur = tr.points[-1].time_from_start
    print('   %-34s ✅ %d 点 %.2fs' % (label, len(tr.points),
                                       dur.sec + dur.nanosec * 1e-9))
    return True


def main():
    rclpy.init()
    node = Node('cart_probe')
    fk_cli = node.create_client(GetPositionFK, '/compute_fk')
    mp_cli = node.create_client(GetMotionPlan, '/plan_kinematic_path')
    sv_cli = node.create_client(GetStateValidity, '/check_state_validity')
    for n, c in (('/compute_fk', fk_cli), ('/plan_kinematic_path', mp_cli)):
        if not c.wait_for_service(timeout_sec=15.0):
            print('FAIL %s 不可用' % n)
            return 1

    start = read_js(node) or [0.0] * 5
    print('实际当前状态: %s' % (['%+.4f' % v for v in start],))
    print('')

    ref = fk(node, fk_cli, Q_REF)
    if ref is None:
        print('FAIL 参考 FK 失败')
        return 1
    xyz = (ref.position.x, ref.position.y, ref.position.z)
    print('参考关节角 %s' % (Q_REF,))
    print('  → 三维坐标 x=%+.4f y=%+.4f z=%+.4f' % xyz)
    print('')

    # ── V0 参考状态合法性 ──
    print('V0  参考状态合法性')
    if sv_cli.wait_for_service(timeout_sec=10.0):
        r = GetStateValidity.Request()
        r.robot_state = rstate(Q_REF)
        r.group_name = GROUP
        res = call(node, sv_cli, r, 10.0)
        if res is None:
            print('   ❌ 服务超时')
        else:
            print('   valid=%s  contacts=%d  %s'
                  % (res.valid, len(res.contacts),
                     '' if res.valid else '← 参考姿态本身就自碰撞！'))
    else:
        print('   （/check_state_validity 不可用，跳过）')

    print('')
    print('V1/V2/V3  规划对比')

    # ── V1 纯位置 10mm ──
    tryplan(node, mp_cli, start, pos_c(xyz, 0.010), 'V1 纯位置 10mm')

    # ── V2 纯位置 50mm ──
    tryplan(node, mp_cli, start, pos_c(xyz, 0.050), 'V2 纯位置 50mm')

    # ── V3 位置 + 姿态（姿态来自参考 FK）──
    c = pos_c(xyz, 0.010)
    c.orientation_constraints = [ori_c(ref, 0.10)]
    tryplan(node, mp_cli, start, c, 'V3 位置10mm + 姿态±0.1')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
