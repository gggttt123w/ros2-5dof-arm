#!/usr/bin/env python3
"""MoveIt2 规划层（无 RViz 版）

板子没显示屏，所以这里【不启动 rviz2】。
需要可视化时在 PC 上跑 RViz，通过 DDS 连过来。

启动内容：
    move_group          ← MoveIt2 的核心（IK / 碰撞检测 / 规划 / 时间参数化）
    static TF world→base_footprint

⚠️ 前提：ros2_control 那套要先起来
      ros2 launch CanServer arm_control.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("my_robot_arm",
                             package_name="my_robot_arm_moveit_config")
        # URDF 用 description 包里那份（不重复维护）
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("my_robot_arm_description"),
                "urdf", "robot_arm.urdf.xacro"),
            mappings={},
        )
        # ★★★ 只加载 ompl，【必须】带 load_all=False ★★★
        #     load_all 默认为 True，会把 MoveIt 自带的 pipeline
        #     （pilz_industrial_motion_planner / chomp 等）也加进来，
        #     接着 to_moveit_configs() 就会去找 pilz_cartesian_limits.yaml
        #     —— 文件不存在 → ParameterBuilderFileNotFoundError。
        #     我们是 5-DOF，OMPL 的 RRTConnect 足够，不需要 pilz。
        .planning_pipelines(pipelines=["ompl"], load_all=False)
        .to_moveit_configs()
    )

    return LaunchDescription([
        # world → base_footprint 的静态 TF
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0",
                       "world", "base_footprint"],
            output="screen",
        ),

        # ★ MoveIt2 核心（纯计算，不需要 X server）
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict()],
        ),

        # ❌ 不启动 rviz2 / moveit_ros_visualization
    ])
