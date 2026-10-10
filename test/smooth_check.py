#!/usr/bin/env python3
"""顺滑度对比测试：同一条慢速轨迹，走多次，量测「运动是否连续」

判断顺滑的客观指标：在轨迹执行期间，以 20Hz 采样 /joint_states，
看【关节速度】的抖动。一卡一卡时速度会忽大忽小（走走停停）。

    python3 smooth_check.py            默认测 joint1 转 0.8 rad / 6s
    python3 smooth_check.py 0.5 4      转 0.5 rad / 4s
"""
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# ★ arm_controller 只管这 5 个关节（gripper 是另一个控制器）
JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
ACTION = '/arm_controller/follow_joint_trajectory'


def main():
    argv = sys.argv[1:]
    delta = float(argv[0]) if argv else 0.8
    dur = float(argv[1]) if len(argv) > 1 else 6.0

    rclpy.init()
    node = Node('smooth_check')

    samples = []
    cur = {'q': None}

    def cb(m):
        try:
            q = [m.position[m.name.index(j)] for j in JOINTS]
        except (ValueError, IndexError):
            return
        cur['q'] = q
        samples.append((time.time(), q))

    node.create_subscription(JointState, '/joint_states', cb, 50)
    while cur['q'] is None:
        rclpy.spin_once(node, timeout_sec=0.5)

    start = list(cur['q'])
    target = list(start)
    target[0] += delta
    print('起始: %s' % (['%+.3f' % v for v in start],))
    print('目标: %s   时长 %.1fs' % (['%+.3f' % v for v in target], dur))
    print('')

    cli = ActionClient(node, FollowJointTrajectory, ACTION)
    if not cli.wait_for_server(timeout_sec=8.0):
        print('FAIL 找不到 %s' % ACTION)
        return 1

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS
    goal.trajectory.points = [
        JointTrajectoryPoint(positions=start,
                             time_from_start=Duration(sec=0, nanosec=0)),
        JointTrajectoryPoint(positions=target,
                             time_from_start=Duration(sec=int(dur),
                                                      nanosec=int((dur % 1) * 1e9))),
    ]

    samples.clear()
    t0 = time.time()
    fut = cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
    gh = fut.result()
    if gh is None or not gh.accepted:
        print('FAIL 目标被拒绝')
        return 1
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=int(dur) + 20)
    code = rf.result().result.error_code

    # 边转边采样
    while time.time() - t0 < dur + 2.5:
        rclpy.spin_once(node, timeout_sec=0.05)

    print('error_code = %s' % code)
    print('到位后: %s' % (['%+.3f' % v for v in cur['q']],))
    print('')

    # ── 用 joint1 的位移序列算「速度抖动」──
    traj = [(t, q[0]) for t, q in samples if t >= t0 - 0.5]
    if len(traj) < 10:
        print('采样太少（%d），无法评估' % len(traj))
        return 0

    vels = []
    for (t1, p1), (t2, p2) in zip(traj, traj[1:]):
        dt = t2 - t1
        if dt > 1e-6:
            vels.append((t2, (p2 - p1) / dt))

    # 只统计运动区间（速度明显非零的部分）
    moving = [(t, v) for t, v in vels if abs(v) > 0.005]
    if len(moving) < 5:
        print('几乎没有运动，无法评估（采样 %d，速度点 %d）' % (len(traj), len(vels)))
        return 0

    vs = [v for _, v in moving]
    mean = statistics.mean(vs)
    sd = statistics.pstdev(vs)
    cv = sd / abs(mean) if mean else 0.0

    # 反向检验：速度变号的次数（走走停停会让符号反复跳）
    sign_changes = sum(1 for a, b in zip(vs, vs[1:]) if a * b < 0)
    # 速度接近 0 的样本占比（停顿）
    stalls = sum(1 for v in vs if abs(v) < 0.02)

    print('════ 顺滑度指标（joint1）════')
    print('  采样点数          %d' % len(traj))
    print('  运动区间样本      %d' % len(moving))
    print('  平均速度          %+.4f rad/s' % mean)
    print('  速度标准差        %.4f rad/s' % sd)
    print('  ★ 变异系数 CV     %.3f        （越小越顺滑）' % cv)
    print('  ★ 停顿样本占比     %.1f%%      （速度 < 0.02 rad/s）' % (100.0 * stalls / len(vs)))
    print('  速度反向次数      %d          （走走停停会反复跳）' % sign_changes)
    print('')
    print('  参考：CV < 0.3 算顺滑，> 0.6 就是明显的走走停停')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
