#!/usr/bin/env python3
"""发关节轨迹到 arm_controller（带范围自检）

用法:
    ./send_traj.py j1 j2 j3 j4 j5 gripper [时长秒]
    ./send_traj.py --limits                打印关节限位表
    ./send_traj.py --home [时长秒]          全部回中位（限位中点）

  某个关节写  -  表示【保持当前位置不动】

示例:
    ./send_traj.py 0.5 0 0 0 0 0 2         joint1 → 0.5 rad，2 秒
    ./send_traj.py 0.5 - - - - - 2         只动 joint1，其余保持
    ./send_traj.py 0 0 0 0 0 0.015 1       夹爪 → 0.015（范围 0~0.02）
    ./send_traj.py --home 3                全部回中位
"""
import sys

# ⚠️ ROS 的 import 放在 main() 里（惰性）——
#    这样 --help / --limits / 范围自检 不需要 source ROS 也能用。

# ──────────────────────────────────────────────────────────
# 关节定义  ⚠️ 改 xacro 的 pos_min/pos_max/dir 时这里要同步
# ──────────────────────────────────────────────────────────
JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5',
          'gripper_finger_left_joint']

LIMITS = [
    (-1.5708, 1.5708),      # joint1
    (-1.5708, 1.5708),      # joint2
    (-1.5708, 1.5708),      # joint3
    (-1.5708, 1.5708),      # joint4
    (-3.1416, 3.1416),      # joint5
    (0.0000,  0.0200),      # gripper_finger_left_joint
]

DIRS = [+1, +1, -1, -1, +1, -1]        # 和 xacro 的 <param name="dir"> 一致

ACTION = '/arm_controller/follow_joint_trajectory'

ERRORS = {
     0: 'SUCCESSFUL',
    -1: 'INVALID_GOAL',
    -2: 'INVALID_JOINTS',
    -3: 'OLD_HEADER_TIMESTAMP',
    -4: 'PATH_TOLERANCE_VIOLATED   ← 路径中偏离容差（命令超范围/舵机跟不上）',
    -5: 'GOAL_TOLERANCE_VIOLATED   ← 终点没进容差',
}


def print_limits():
    print()
    print(f"  {'#':<3}{'关节':<28}{'最小':>10}{'最大':>10}{'方向':>6}")
    print("  " + "─" * 58)
    for i, (name, (lo, hi), d) in enumerate(zip(JOINTS, LIMITS, DIRS)):
        print(f"  {i:<3}{name:<28}{lo:>10.4f}{hi:>10.4f}{d:>+6d}")
    print()
    print("  提示：方向 dir=-1 的关节，命令正值时实机往反方向转")
    print()


def parse_args(argv):
    """返回 (target, dur)；出错时返回 (None, None)"""
    if not argv or argv[0] in ('-h', '--help'):
        print(__doc__)
        return None, None

    if argv[0] == '--limits':
        print_limits()
        return None, None

    if argv[0] == '--home':
        dur = float(argv[1]) if len(argv) > 1 else 3.0
        target = [(lo + hi) / 2.0 for lo, hi in LIMITS]
        return target, dur

    if len(argv) < 6:
        print(f"❌ 需要 6 个关节角，只给了 {len(argv)} 个")
        print(__doc__)
        return None, None

    # '-' 表示保持当前位置（下面用 cur 替换）
    target = []
    for i, x in enumerate(argv[:6]):
        if x in ('-', '~', 'x', 'X'):
            target.append(None)
            continue
        try:
            target.append(float(x))
        except ValueError:
            print(f"❌ 第 {i + 1} 个参数 {x!r} 不是数字（用 - 表示保持不动）")
            return None, None

    dur = float(argv[6]) if len(argv) > 6 else 3.0
    if dur <= 0:
        print(f"❌ 时长必须 > 0，给了 {dur}")
        return None, None
    return target, dur


def check_limits(target, cur=None):
    """范围自检（target 里的 None 表示保持当前位置，跳过检查）。
    返回 True 表示全部合法"""
    bad = []
    for i, (name, v, (lo, hi)) in enumerate(zip(JOINTS, target, LIMITS)):
        if v is None:
            continue
        if not (lo <= v <= hi):
            bad.append((i, name, v, lo, hi))

    if not bad:
        return True

    print()
    print("  ❌ 命令超出关节限位，已中止（没有发送）")
    print()
    print(f"  {'关节':<28}{'你给的':>12}{'合法范围':>24}{'超出':>12}")
    print("  " + "─" * 78)
    for i, name, v, lo, hi in bad:
        over = v - hi if v > hi else v - lo
        print(f"  {name:<28}{v:>12.4f}{f'[{lo:.4f}, {hi:.4f}]':>24}{over:>+12.4f}")
    print()
    print("  用  ./send_traj.py --limits  查看全部限位")
    print()
    return False


def main():
    target, dur = parse_args(sys.argv[1:])
    if target is None:
        sys.exit(0)

    # ★ 发之前先范围自检（不需要 ROS）
    if not check_limits(target):
        sys.exit(1)

    # ── 到这里才需要 ROS ──
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from sensor_msgs.msg import JointState
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from builtin_interfaces.msg import Duration

    rclpy.init()
    node = Node('send_traj')

    # ── ① 读当前位置 ──
    cur = None

    def cb(msg):
        nonlocal cur
        try:
            cur = [msg.position[msg.name.index(j)] for j in JOINTS]
        except (ValueError, IndexError):
            pass

    node.create_subscription(JointState, '/joint_states', cb, 10)
    deadline = node.get_clock().now().nanoseconds + int(5e9)
    while cur is None and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)

    if cur is None:
        print("❌ 5 秒内没收到 /joint_states —— ros2_control 起来了吗？")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # ★ '-' 的地方用当前位置填充（保持不动）
    kept = [JOINTS[i] for i, v in enumerate(target) if v is None]
    target = [c if v is None else v for v, c in zip(target, cur)]
    if kept:
        print(f"  （{', '.join(kept)} 保持当前位置）")

    # ── ② 打印计划 ──
    print()
    print(f"  {'关节':<28}{'当前':>10}{'目标':>10}{'移动':>10}{'速度':>12}")
    print("  " + "─" * 72)
    for name, c, t in zip(JOINTS, cur, target):
        d = t - c
        print(f"  {name:<28}{c:>10.4f}{t:>10.4f}{d:>+10.4f}{d / dur:>+10.3f} rad/s")
    print()
    print(f"  时长 {dur}s")

    # ── ③ 发轨迹 ──
    cli = ActionClient(node, FollowJointTrajectory, ACTION)
    if not cli.wait_for_server(timeout_sec=5.0):
        print(f"❌ 找不到 action server: {ACTION}")
        print("   ros2_control 起来了吗？ ros2 control list_controllers")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS
    goal.trajectory.points = [
        JointTrajectoryPoint(positions=cur,
                             time_from_start=Duration(sec=0, nanosec=0)),
        JointTrajectoryPoint(positions=target,
                             time_from_start=Duration(
                                 sec=int(dur),
                                 nanosec=int((dur % 1) * 1e9))),
    ]

    fut = cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut)
    gh = fut.result()
    if not gh.accepted:
        print("❌ 目标被拒绝（joint_trajectory_controller 不接受）")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    print("  ✅ 已接受，执行中…")

    res_fut = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res_fut)
    code = res_fut.result().result.error_code

    print()
    if code == 0:
        print(f"  ✅ error_code = 0  ({ERRORS[0]})")
    else:
        print(f"  ❌ error_code = {code}")
        print(f"     {ERRORS.get(code, '未知错误码')}")
    print()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if code == 0 else 1)


if __name__ == '__main__':
    main()
