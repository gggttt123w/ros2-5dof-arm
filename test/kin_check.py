#!/usr/bin/env python3
"""用 /compute_fk 校验解析运动学模型

推导（从 URDF）：
    joint1 绕 z 转        base→j2 轴线距离 Z0 = 0.0645 + 0.008 = 0.0725
    joint2/3/4 绕 y 转    L1 = 0.0985 (j2→j3)
                          L2 = 0.0950 (j3→j4)
    link5 原点            L3 = 0.0685 (j4→link5)

    平面内（q1=0 时）：
        a2 = q2
        a3 = q2 + q3
        a4 = q2 + q3 + q4
        x  = L1*sin(a2) + L2*sin(a3) + L3*sin(a4)
        z  = Z0 + L1*cos(a2) + L2*cos(a3) + L3*cos(a4)
    绕 joint1 旋转 q1 后：
        X = x*cos(q1)
        Y = ±x*sin(q1)      ← 符号待定
"""
import math
import random
import sys
import time

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState, MoveItErrorCodes

L1, L2, L3, Z0 = 0.0985, 0.0950, 0.0685, 0.0725
BASE, TIP = 'base_link', 'link5'
JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
LIM = [1.5708, 1.5708, 1.5708, 1.5708, 3.1416]


def model(q, ysign):
    q1, q2, q3, q4, _ = q
    a2, a3, a4 = q2, q2 + q3, q2 + q3 + q4
    x = L1 * math.sin(a2) + L2 * math.sin(a3) + L3 * math.sin(a4)
    z = Z0 + L1 * math.cos(a2) + L2 * math.cos(a3) + L3 * math.cos(a4)
    return x * math.cos(q1), ysign * x * math.sin(q1), z


def main():
    rclpy.init()
    node = Node('kin_check')
    cli = node.create_client(GetPositionFK, '/compute_fk')
    if not cli.wait_for_service(timeout_sec=20.0):
        print('FAIL /compute_fk 不可用')
        return 1

    random.seed(7)
    cases = [[0.0, 0.0, 0.0, 0.0, 0.0],
             [0.0, 0.4, -0.6, 0.3, 0.0],
             [0.5, 0.4, -0.6, 0.3, 0.0],
             [-0.5, 0.4, -0.6, 0.3, 0.0]]
    for _ in range(6):
        cases.append([random.uniform(-l * 0.8, l * 0.8) for l in LIM])

    print('  %-40s %-26s %-26s %s' % ('关节角', 'FK 实测', '模型(ysign=+1)', '误差'))
    print('  ' + '-' * 110)

    worst = {'+1': 0.0, '-1': 0.0}
    for q in cases:
        req = GetPositionFK.Request()
        req.header.frame_id = BASE
        req.fk_link_names = [TIP]
        rs = RobotState()
        rs.joint_state.name = JOINTS
        rs.joint_state.position = list(q)
        req.robot_state = rs
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
        res = fut.result()
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            print('  FK 失败: %s' % q)
            continue
        p = res.pose_stamped[0].pose.position
        errs = {}
        for s, tag in ((1, '+1'), (-1, '-1')):
            mx, my, mz = model(q, s)
            errs[tag] = math.sqrt((mx - p.x) ** 2 + (my - p.y) ** 2 + (mz - p.z) ** 2)
            worst[tag] = max(worst[tag], errs[tag])
        m1 = model(q, 1)
        print('  %-40s (%+.4f,%+.4f,%+.4f)  (%+.4f,%+.4f,%+.4f)  +1:%.5f -1:%.5f'
              % (['%+.2f' % v for v in q], p.x, p.y, p.z, m1[0], m1[1], m1[2],
                 errs['+1'], errs['-1']))

    print('')
    print('  最大误差:  ysign=+1 → %.6f m    ysign=-1 → %.6f m' % (worst['+1'], worst['-1']))
    if worst['+1'] < 1e-4:
        print('  ✅ 模型正确，joint1 用  Y = +x*sin(q1)')
    elif worst['-1'] < 1e-4:
        print('  ✅ 模型正确，joint1 用  Y = -x*sin(q1)')
    else:
        print('  ❌ 模型不对，需要重新推导')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
