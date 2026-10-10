#!/usr/bin/env python3
"""ros2_control 控制层（无 RViz 版）

    robot_state_publisher        URDF → TF + /robot_description
    ros2_control_node            controller_manager + 加载 ArmSystemHardware
    spawner joint_state_broadcaster
    spawner arm_controller       ← 组 arm  (joint1~joint5)
    spawner gripper_controller   ← 组 gripper (gripper_finger_left_joint)

⚠️ 控制器负责的关节集合必须和 MoveIt2 的规划组一致，
   否则 joint_trajectory_controller 会因 allow_partial_joints_goal=false
   拒绝 MoveIt 发来的轨迹（Goal was rejected by server）。
"""
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # ① 展开 URDF（含 <ros2_control> 段）
    xacro_file = os.path.join(
        get_package_share_directory('my_robot_arm_description'),
        'urdf', 'robot_arm.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    # ② 控制器配置
    ctrl_yaml = os.path.join(
        get_package_share_directory('CanServer'),
        'config', 'ros2_controllers.yaml')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen',
        ),

        # ★ 加载硬件接口 + 控制器管理器
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_desc}, ctrl_yaml],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller', '-c', '/controller_manager'],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_controller', '-c', '/controller_manager'],
            output='screen',
        ),
    ])
