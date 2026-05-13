"""
Launch the Piper arm + gripper in MuJoCo (visualization only, no MoveIt).

The MuJoCo physics window opens automatically. Press Space to pause/resume,
right-arrow to single-step. Use headless:=true for servers without a display.

Usage
-----
  ros2 launch agx_arm_mjsim piper_with_gripper_mjsim.launch.py
  ros2 launch agx_arm_mjsim piper_with_gripper_mjsim.launch.py use_rviz:=false
  ros2 launch agx_arm_mjsim piper_with_gripper_mjsim.launch.py headless:=true

Extra sim services (provided by mujoco_ros2_control)
------------------------------------------------------
  ros2 service call /ros2_control_node/set_pause      mujoco_ros2_control_msgs/srv/SetPause      "{paused: true}"
  ros2 service call /ros2_control_node/reset_world    mujoco_ros2_control_msgs/srv/ResetWorld    "{}"
  ros2 service call /ros2_control_node/step_simulation mujoco_ros2_control_msgs/srv/StepSimulation "{steps: 100}"
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz for visualization",
    )
    declare_headless = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run MuJoCo without GUI window (useful for CI/headless servers)",
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("agx_arm_mjsim"), "urdf", "piper_with_gripper_mjsim.urdf.xacro"]
            ),
            " initial_positions_file:=",
            PathJoinSubstitution(
                [FindPackageShare("agx_arm_mjsim"), "config", "initial_positions.yaml"]
            ),
            " headless:=",
            LaunchConfiguration("headless"),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # robot_state_publisher: provides /tf and /tf_static from the URDF
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # mujoco_ros2_control node: runs MuJoCo physics + controller_manager
    # Must use this package's node (slightly modified upstream, to be merged later).
    control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": True},
            PathJoinSubstitution(
                [FindPackageShare("agx_arm_mjsim"), "config", "ros2_controllers.yaml"]
            ),
        ],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("agx_arm_mjsim"), "config", "mjsim.rviz"]
            ),
        ],
        parameters=[robot_description, {"use_sim_time": True}],
        output="log",
    )

    # Sequence: spawn joint_state_broadcaster first, then arm + gripper together.
    delayed_jsb = TimerAction(period=2.0, actions=[jsb_spawner])

    start_controllers_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[arm_spawner, gripper_spawner],
        )
    )

    return LaunchDescription(
        [
            declare_use_rviz,
            declare_headless,
            rsp_node,
            control_node,
            delayed_jsb,
            start_controllers_after_jsb,
            rviz_node,
        ]
    )
