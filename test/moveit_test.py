#!/usr/bin/env python3
"""MoveIt2 端到端测试：FK → IK → 关节空间规划 → ★笛卡尔(三维坐标)规划★

用法:
    python3 moveit_test.py              只做 FK/IK
    python3 moveit_test.py plan         再加关节空间规划
    python3 moveit_test.py cart         再加【三维坐标】规划   ← 核心目标
    python3 moveit_test.py cart x y z   指定三维坐标

判定：
    ① /compute_fk 返回末端位姿          → move_group + URDF/SRDF 都对
    ② /compute_ik 能解回来              → IK 可用（5-DOF 的关键）
    ③ 关节空间规划 SUCCESS              → OMPL 通了
    ④ ★三维坐标规划 SUCCESS★            → 「移动到任意可到达的三维坐标」
"""
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetMotionPlan
from moveit_msgs.msg import (
    PositionIKRequest, RobotState, MoveItErrorCodes,
    Constraints, JointConstraint, MotionPlanRequest,
    PositionConstraint, BoundingVolume,
)

GROUP = 'arm'
TIP = 'link5'
BASE = 'base_link'
JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']

ERR = {
    MoveItErrorCodes.SUCCESS: 'SUCCESS',
    MoveItErrorCodes.FAILURE: 'FAILURE',
    MoveItErrorCodes.NO_IK_SOLUTION: 'NO_IK_SOLUTION',
    MoveItErrorCodes.TIMED_OUT: 'TIMED_OUT',
    MoveItErrorCodes.PLANNING_FAILED: 'PLANNING_FAILED',
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: 'INVALID_GOAL_CONSTRAINTS',
    MoveItErrorCodes.INVALID_GROUP_NAME: 'INVALID_GROUP_NAME',
    MoveItErrorCodes.START_STATE_IN_COLLISION: 'START_STATE_IN_COLLISION',
    MoveItErrorCodes.INVALID_ROBOT_STATE: 'INVALID_ROBOT_STATE',
    MoveItErrorCodes.GOAL_IN_COLLISION: 'GOAL_IN_COLLISION',
}


def err_name(c):
    return ERR.get(c, 'code=%s' % c)


def robot_state(seed):
    rs = RobotState()
    rs.joint_state.name = JOINTS
    rs.joint_state.position = list(seed)
    return rs


def call(node, cli, req, timeout=20.0):
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    return fut.result()


def read_joint_states(node, timeout=6.0):
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


def do_fk(node, cli, seed):
    req = GetPositionFK.Request()
    req.header.frame_id = BASE
    req.fk_link_names = [TIP]
    req.robot_state = robot_state(seed)
    res = call(node, cli, req, 10.0)
    if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
        return None, (err_name(res.error_code.val) if res else 'timeout')
    return res.pose_stamped[0].pose, None


def do_ik(node, cli, pose, seed):
    ps = PoseStamped()
    ps.header.frame_id = BASE
    ps.pose = pose

    req = GetPositionIK.Request()
    req.ik_request = PositionIKRequest()
    req.ik_request.group_name = GROUP
    req.ik_request.ik_link_name = TIP
    req.ik_request.pose_stamped = ps
    req.ik_request.avoid_collisions = False
    req.ik_request.robot_state = robot_state(seed)
    req.ik_request.timeout.sec = 2

    res = call(node, cli, req, 20.0)
    if res is None:
        return None, 'timeout'
    if res.error_code.val != MoveItErrorCodes.SUCCESS:
        return None, err_name(res.error_code.val)
    d = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
    return [d.get(j, float('nan')) for j in JOINTS], None


def plan(node, cli, start, goal_constraints, label):
    req = GetMotionPlan.Request()
    mr = MotionPlanRequest()
    mr.group_name = GROUP
    mr.num_planning_attempts = 5
    mr.allowed_planning_time = 5.0
    mr.start_state = robot_state(start)
    # 缩放因子：只用到限制的 30%，轨迹更柔和（对应「丝滑」）
    mr.max_velocity_scaling_factor = 0.3
    mr.max_acceleration_scaling_factor = 0.3
    mr.goal_constraints = [goal_constraints]
    req.motion_plan_request = mr

    res = call(node, cli, req, 40.0)
    if res is None:
        print('%s FAIL timeout' % label)
        return None
    code = res.motion_plan_response.error_code.val
    if code != MoveItErrorCodes.SUCCESS:
        print('%s FAIL %s' % (label, err_name(code)))
        return None

    traj = res.motion_plan_response.trajectory.joint_trajectory
    dur = traj.points[-1].time_from_start
    secs = dur.sec + dur.nanosec * 1e-9
    print('%s OK   %d points, %.2fs' % (label, len(traj.points), secs))
    print('     start %s' % (['%+.3f' % v for v in traj.points[0].positions],))
    print('     end   %s' % (['%+.3f' % v for v in traj.points[-1].positions],))
    return traj


def joint_constraints(target):
    c = Constraints()
    for name, val in zip(JOINTS, target):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = val
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        c.joint_constraints.append(jc)
    return c


def position_constraint(xyz, tol=0.01):
    """把 link5 的原点约束到 (x,y,z) 附近 tol 米的球内
       —— 这就是「移动到任意可到达的三维坐标」"""
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


def main():
    argv = sys.argv[1:]
    mode = argv[0] if argv else ''
    rclpy.init()
    node = Node('moveit_test')

    fk_cli = node.create_client(GetPositionFK, '/compute_fk')
    ik_cli = node.create_client(GetPositionIK, '/compute_ik')
    mp_cli = node.create_client(GetMotionPlan, '/plan_kinematic_path')
    for name, cli in (('/compute_fk', fk_cli), ('/compute_ik', ik_cli)):
        if not cli.wait_for_service(timeout_sec=15.0):
            print('FAIL %s unavailable - is move_group running?' % name)
            return 1

    # ---- (1) FK / (2) IK ----
    seed = [0.0, 0.4, -0.6, 0.3, 0.0]
    pose, e = do_fk(node, fk_cli, seed)
    if pose is None:
        print('(1) FK  FAIL %s' % e)
        return 1
    p = pose.position
    print('(1) FK  OK   seed=%s' % (seed,))
    print('     tip pos  x=%+.4f  y=%+.4f  z=%+.4f' % (p.x, p.y, p.z))

    sol, e = do_ik(node, ik_cli, pose, seed)
    if sol is None:
        print('(2) IK  FAIL %s' % e)
        return 1
    print('(2) IK  OK   %s  maxdiff %.4f rad'
          % (['%+.4f' % v for v in sol],
             max(abs(a - b) for a, b in zip(sol, seed))))

    if mode not in ('plan', 'cart'):
        return 0

    if not mp_cli.wait_for_service(timeout_sec=15.0):
        print('FAIL /plan_kinematic_path unavailable')
        return 1

    # 起点：优先用机械臂的真实当前状态
    real = read_joint_states(node)
    if real:
        print('     real state: %s' % (['%+.4f' % v for v in real],))
        start = real
    else:
        print('     WARN no /joint_states, using seed as start')
        start = seed

    # ---- (3) joint-space planning ----
    target = [0.5, 0.2, -0.3, 0.1, 0.4]
    plan(node, mp_cli, start, joint_constraints(target), '(3) joint-space plan')

    if mode != 'cart':
        return 0

    # ---- (4) CARTESIAN (3D coordinate) planning ----
    # 先用 FK 从一个已知好的关节角算出「一定可达」的笛卡尔点，
    # 再只给坐标，让 MoveIt 自己解 IK + 规划。
    q_ref = [0.4, 0.6, -0.8, 0.3, 0.2]
    ref_pose, e = do_fk(node, fk_cli, q_ref)
    if ref_pose is None:
        print('(4) FK ref failed: %s' % e)
        return 1

    if len(argv) >= 4:
        xyz = (float(argv[1]), float(argv[2]), float(argv[3]))
        src = 'from command line'
    else:
        xyz = (ref_pose.position.x, ref_pose.position.y, ref_pose.position.z)
        src = 'FK of joints %s' % (q_ref,)

    print('')
    print('(4) CARTESIAN target  x=%+.4f y=%+.4f z=%+.4f   (%s)'
          % (xyz[0], xyz[1], xyz[2], src))

    traj = plan(node, mp_cli, start, position_constraint(xyz, 0.01), '    plan')
    if traj is None:
        return 1

    # 用 FK 校验终点到底落在哪
    end_q = list(traj.points[-1].positions)
    end_pose, e = do_fk(node, fk_cli, end_q)
    if end_pose:
        ep = end_pose.position
        d = ((ep.x - xyz[0]) ** 2 + (ep.y - xyz[1]) ** 2 + (ep.z - xyz[2]) ** 2) ** 0.5
        print('     reached  x=%+.4f y=%+.4f z=%+.4f' % (ep.x, ep.y, ep.z))
        print('     error    %.1f mm   %s'
              % (d * 1000, 'OK (<10mm)' if d < 0.01 else 'FAIL'))

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
