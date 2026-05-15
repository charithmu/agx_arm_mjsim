"""
Launch Piper + gripper in MuJoCo with MoveIt2.

The MuJoCo physics window opens alongside RViz + MotionPlanning panel.
MoveIt borrows SRDF / kinematics / planning pipelines from agx_arm_moveit
(upstream, not copied) and overrides only trajectory_execution to point at
the sim controller action servers.

Usage
-----
  ros2 launch agx_arm_mjsim piper_with_gripper_moveit_mjsim.launch.py
  ros2 launch agx_arm_mjsim piper_with_gripper_moveit_mjsim.launch.py headless:=true
"""

import ast
import sys
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

# Borrow agx_arm_moveit's config builder so SRDF, kinematics and planning
# pipelines stay in sync with the upstream package without copying anything.
sys.path.insert(
    0, str(Path(get_package_share_directory("agx_arm_moveit")) / "launch")
)
from _moveit_config_builder import build_moveit_config  # noqa: E402

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_moveit(context):
    pkg_mjsim = get_package_share_directory("agx_arm_mjsim")
    pkg_moveit = get_package_share_directory("agx_arm_moveit")
    tcp_offset = ast.literal_eval(LaunchConfiguration("tcp_offset").perform(context))
    tcp_offset_xyz = f"{tcp_offset[0]} {tcp_offset[1]} {tcp_offset[2]}"
    tcp_offset_rpy = f"{tcp_offset[3]} {tcp_offset[4]} {tcp_offset[5]}"

    # All MoveIt config (SRDF, kinematics, planning pipelines, joint limits)
    # comes from agx_arm_moveit via the shared builder – no duplication here.
    moveit_config = build_moveit_config(context)

    # Override trajectory execution only: sim uses ros2_control action servers.
    with open(Path(pkg_mjsim) / "config" / "moveit_controllers.yaml") as f:
        moveit_config.trajectory_execution = yaml.safe_load(f)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                # Joints that settle just outside their limit boundary are still valid.
                "start_state_max_bounds_error": 0.01,
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        # Use agx_arm_moveit's RViz config which includes the MotionPlanning panel.
        arguments=["-d", str(Path(pkg_moveit) / "config" / "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("agx_arm_mjsim"),
                        "launch",
                        "piper_with_gripper_mjsim.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_rviz": "false",
            "headless": LaunchConfiguration("headless"),
            "tcp_offset_xyz": tcp_offset_xyz,
            "tcp_offset_rpy": tcp_offset_rpy,
        }.items(),
    )

    return [sim_launch, move_group_node, rviz_node]


def generate_launch_description():
    return LaunchDescription(
        [
            # Declared so build_moveit_config() can read them from the context.
            DeclareLaunchArgument("arm_type", default_value="piper"),
            DeclareLaunchArgument("effector_type", default_value="agx_gripper"),
            DeclareLaunchArgument("revo2_type", default_value="left"),
            DeclareLaunchArgument(
                "tcp_offset",
                default_value="[0.0, 0.0, 0.1358, 0.0, 0.0, 0.0]",
                description="TCP offset [x, y, z, rx, ry, rz] in metres/radians. Defaults to the midpoint between the gripper jaws.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run MuJoCo without GUI window.",
            ),
            OpaqueFunction(function=_launch_moveit),
        ]
    )
