#!/usr/bin/env python3
"""顺滑度对比：纯串口(裸 CAN) vs MoveIt2 流式下发

用 raw SocketCAN 直接和 STM32 说话，20Hz 高频查询舵机位置，
拿到真实运动曲线，再算速度抖动指标。

    python3 can_smooth.py direct            纯串口：一条 T=6s 的命令
    python3 can_smooth.py direct 3.0        改时长

对比时：
    ① 先停掉 ros2_control，跑 direct → 这是"你说的那个丝滑基线"
    ② 再起 ros2_control，跑 jtc     → 这是 MoveIt2 那条路
    ③ 比较两者的 CV（变异系数，越小越顺滑）
"""
import math
import socket
import statistics
import struct
import sys
import time

CAN_IF = 'can0'
ID_DOWN = 0x104       # 本机 → STM32
ID_UP = 0x200         # STM32 → 本机
ASK_SETPOS = 1
ASK_STATUS = 2

# joint1 的标定（和 xacro 一致）
J1_ID = 0
J1_SERVO_MIN, J1_SERVO_MAX = 500, 2500
J1_POS_MIN, J1_POS_MAX = -1.5708, 1.5708
J1_DIR = +1


def angle_to_servo(a):
    sp = J1_SERVO_MAX - J1_SERVO_MIN
    ap = J1_POS_MAX - J1_POS_MIN
    t = (a - J1_POS_MIN) / ap
    if J1_DIR < 0:
        t = 1.0 - t
    t = max(0.0, min(1.0, t))
    return int(J1_SERVO_MIN + t * sp)


def servo_to_angle(p):
    sp = J1_SERVO_MAX - J1_SERVO_MIN
    ap = J1_POS_MAX - J1_POS_MIN
    t = (p - J1_SERVO_MIN) / sp
    if J1_DIR < 0:
        t = 1.0 - t
    t = max(0.0, min(1.0, t))
    return J1_POS_MIN + t * ap


def can_open():
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((CAN_IF,))
    s.settimeout(0.0)
    return s


def send(s, can_id, data):
    pad = bytes(data) + b'\x00' * (8 - len(data))
    s.send(struct.pack('=IB3x8s', can_id, len(data), pad))


def drain(s, timeout=0.15):
    """收掉所有在途帧，返回 {id: position}"""
    got = {}
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            f = s.recv(16)
        except BlockingIOError:
            time.sleep(0.002)
            continue
        if len(f) < 16:
            continue
        can_id, dlc = struct.unpack('=IB3x', f[:8])
        d = f[8:16]
        if (can_id & 0x1FFFFFFF) == ID_UP and dlc >= 3:
            sid = d[0]
            pos = d[1] | (d[2] << 8)
            got[sid] = pos
    return got


def query(s, sid):
    send(s, ID_DOWN, [sid, ASK_STATUS, 0, 0, 0, 0, 0, 0])


def read_pos(s, sid, tries=3):
    for _ in range(tries):
        query(s, sid)
        got = drain(s, 0.035)
        if sid in got:
            return got[sid]
    return None


def metrics(track, label, k=8):
    """track: [(t, angle)]。用大小为 k 的窗口做局部线性拟合估速度，
       避免采样间隔不均造成的假抖动。"""
    if len(track) < 2 * k + 2:
        print('  %s: 样本太少 (%d)' % (label, len(track)))
        return None

    t0 = track[0][0]
    ts = [t - t0 for t, _ in track]
    xs = [a for _, a in track]

    vels = []
    for i in range(k, len(track) - k):
        # 局部窗口最小二乘斜率
        n = 2 * k + 1
        tw = ts[i - k:i + k + 1]
        xw = xs[i - k:i + k + 1]
        mt = sum(tw) / n
        mx = sum(xw) / n
        num = sum((a - mt) * (b - mx) for a, b in zip(tw, xw))
        den = sum((a - mt) ** 2 for a in tw)
        if den > 1e-12:
            vels.append((ts[i], num / den))

    if not vels:
        print('  %s: 算不出速度' % label)
        return None

    total = abs(xs[-1] - xs[0])
    vmax = max(abs(v) for _, v in vels)
    moving = [(t, v) for t, v in vels if abs(v) > 0.15 * vmax]

    if len(moving) < 3:
        print('  %s: 运动段太短' % label)
        return None

    vs = [v for _, v in moving]
    mean = statistics.mean(vs)
    sd = statistics.pstdev(vs)
    cv = sd / abs(mean) if mean else 0.0
    stalls = sum(1 for v in vs if abs(v) < 0.15 * vmax)
    rev = sum(1 for a, b in zip(vs, vs[1:]) if a * b < 0)

    print('  %-14s 位移 %+.3f rad  样本 %3d  运动段 %3d' % (label, total, len(track), len(moving)))
    print('  %-14s 平均速度 %+.3f  峰值 %5.3f  标准差 %.3f' % ('', mean, vmax, sd))
    print('  %-14s ★ CV %.3f   停顿占比 %.1f%%   速度反向 %d 次'
          % ('', cv, 100.0 * stalls / len(vs), rev))
    return {'cv': cv, 'stall': 100.0 * stalls / len(vs), 'rev': rev,
            'mean': mean, 'vmax': vmax, 'n': len(track)}


def run_home(target_a=0.0, dur=3.0):
    """把 joint1 慢慢挪到一个安全的中点，作为基线起点"""
    s = can_open()
    p0 = read_pos(s, J1_ID)
    if p0 is None:
        print('FAIL 读不到舵机位置')
        return 1
    a0 = servo_to_angle(p0)
    pT = angle_to_servo(target_a)
    print('════ 归位：joint1 %.4f → %.4f rad（servo %d → %d）════'
          % (a0, target_a, p0, pT))
    send(s, ID_DOWN, [J1_ID, ASK_SETPOS,
                      pT & 0xFF, (pT >> 8) & 0xFF,
                      int(dur * 1000) & 0xFF, (int(dur * 1000) >> 8) & 0xFF, 0, 0])
    t0 = time.time()
    last = p0
    while time.time() - t0 < dur + 2.0:
        p = read_pos(s, J1_ID, tries=1)
        if p is not None:
            last = p
        time.sleep(0.1)
    print('  到位 servo=%d → %.4f rad' % (last, servo_to_angle(last)))
    print('')
    return 0


def run_direct(dur):
    s = can_open()
    p0 = read_pos(s, J1_ID)
    if p0 is None:
        print('FAIL 读不到舵机 %d 的位置 —— CAN 通吗？' % J1_ID)
        return 1
    a0 = servo_to_angle(p0)
    # ★ 方向自适应：+0.8 会撞限位就往反方向走
    delta = 0.8 if (a0 + 0.8) <= J1_POS_MAX - 0.05 else -0.8
    aT = a0 + delta
    pT = angle_to_servo(aT)
    t_ms = int(dur * 1000)

    print('════ 纯串口基线（完全绕开 ros2_control）════')
    print('  当前 servo=%d → %.4f rad' % (p0, a0))
    print('  目标 servo=%d → %.4f rad' % (pT, aT))
    print('  方向 %+.1f rad' % delta)
    print('  一条命令: 104#%02X%02X%02X%02X%02X%02X0000   (T=%d ms)'
          % (J1_ID, ASK_SETPOS, pT & 0xFF, (pT >> 8) & 0xFF,
             t_ms & 0xFF, (t_ms >> 8) & 0xFF, t_ms))
    print('')

    # 发一条长 T 命令（这就是「直接通过串口发送坐标」）
    send(s, ID_DOWN, [J1_ID, ASK_SETPOS,
                      pT & 0xFF, (pT >> 8) & 0xFF,
                      t_ms & 0xFF, (t_ms >> 8) & 0xFF, 0, 0])

    track = []
    t0 = time.time()
    while time.time() - t0 < dur + 1.5:
        p = read_pos(s, J1_ID, tries=1)
        if p is not None:
            track.append((time.time() - t0, servo_to_angle(p)))
        time.sleep(0.008)     # 目标 20Hz 采样

    print('════ 结果 ════')
    r = metrics(track, '纯串口')
    return 0


def run_jtc(dur):
    """MoveIt2/ros2_control 那条路：发一条 FollowJointTrajectory，
       同时用裸 CAN 高频采样"""
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from builtin_interfaces.msg import Duration

    s = can_open()
    p0 = read_pos(s, J1_ID)
    if p0 is None:
        print('FAIL 读不到舵机位置')
        return 1
    a0 = servo_to_angle(p0)
    delta = 0.8 if (a0 + 0.8) <= J1_POS_MAX - 0.05 else -0.8
    aT = a0 + delta

    rclpy.init()
    node = Node('can_smooth')
    cli = ActionClient(node, FollowJointTrajectory,
                       '/arm_controller/follow_joint_trajectory')
    if not cli.wait_for_server(timeout_sec=8.0):
        print('FAIL 找不到 arm_controller —— ros2_control 起了吗？')
        return 1

    J = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
    start = [a0, 0.0, 0.0, 0.0, 0.0]
    target = [aT, 0.0, 0.0, 0.0, 0.0]

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = J
    goal.trajectory.points = [
        JointTrajectoryPoint(positions=start, time_from_start=Duration(sec=0)),
        JointTrajectoryPoint(positions=target,
                             time_from_start=Duration(sec=int(dur),
                                                      nanosec=int((dur % 1) * 1e9))),
    ]

    print('════ ros2_control + MoveIt2 链路 ════')
    print('  joint1 %.4f → %.4f rad，轨迹时长 %.1fs' % (a0, aT, dur))
    print('')

    fut = cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
    gh = fut.result()
    if gh is None or not gh.accepted:
        print('FAIL 目标被拒绝')
        return 1

    track = []
    t0 = time.time()
    rf = gh.get_result_async()
    while time.time() - t0 < dur + 1.5:
        rclpy.spin_once(node, timeout_sec=0.0)
        p = read_pos(s, J1_ID, tries=1)
        if p is not None:
            track.append((time.time() - t0, servo_to_angle(p)))
        time.sleep(0.008)

    print('════ 结果 ════')
    r = metrics(track, 'ros2_control')
    node.destroy_node()
    rclpy.shutdown()
    return 0


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'direct'
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0

    if mode == 'home':
        return run_home(0.0, dur)
    if mode == 'direct':
        return run_direct(dur)
    if mode == 'jtc':
        return run_jtc(dur)
    print(__doc__)
    return 1


if __name__ == '__main__':
    sys.exit(main())
