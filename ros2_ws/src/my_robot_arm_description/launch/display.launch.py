#!/usr/bin/env python3
"""在 RViz2 中显示 my_robot_arm。

用法：
    ros2 launch my_robot_arm_description display.launch.py
    ros2 launch my_robot_arm_description display.launch.py use_gui:=false   # 不给关节滑条
    ros2 launch my_robot_arm_description display.launch.py use_rviz:=false  # 只发 description/TF
    ros2 launch my_robot_arm_description display.launch.py use_sim_time:=true
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_arm_description')
    default_xacro = os.path.join(pkg_share, 'urdf', 'robot_arm.urdf.xacro')
    default_rviz = os.path.join(pkg_share, 'rviz', 'display.rviz')

    # 在 launch 时展开 xacro，直接把结果字符串交给 robot_state_publisher，
    # 这样不依赖 xacro 可执行文件，也不会产生中间文件。
    robot_description = xacro.process_file(default_xacro).toxml()

    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui', default_value='true',
            description='启动 joint_state_publisher_gui，用滑条手动拖动各关节'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='是否同时启动 RViz2'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='使用 /clock 仿真时间'),
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz,
            description='RViz2 配置文件路径'),

        # 发布 /robot_description 与整棵 TF 树
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
        ),

        # 关节值来源二选一：GUI 滑条 / 无界面发布器
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=UnlessCondition(use_gui),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
