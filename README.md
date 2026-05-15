# agx_arm_mjsim

MuJoCo simulation for the **Piper 6-DOF arm with parallel-jaw gripper**.  
Supports standalone visualization and full MoveIt2 motion planning.

The MJCF model is sourced from [MuJoCo Menagerie (agilex_piper)](https://github.com/google-deepmind/mujoco_menagerie/tree/main/agilex_piper)
via the `robot_descriptions` Python package.  Joint names and actuators are
adapted to match the ros2_control / MoveIt convention used in this workspace
(`gripper_joint1`, `gripper_joint2`) so the same SRDF, controllers, and
motion-planning code work identically across MuJoCo and Gazebo.

## Prerequisites

| Dependency | Version |
|---|---|
| ROS 2 | Jazzy |
| mujoco_ros2_control | 1.0+ |
| `agx_arm_description` | – |
| `agx_arm_moveit` | – |
| MuJoCo Python | 3.x (workspace venv) |

## Build

```bash
cd <workspace>
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate          # workspace venv with MuJoCo installed
colcon build --packages-select agx_arm_mjsim
source install/setup.bash
```

## Launch

### Visualization only (MuJoCo + RViz)

```bash
ros2 launch agx_arm_mjsim piper_with_gripper_mjsim.launch.py
```

Optional arguments:
- `use_rviz:=false` — skip RViz
- `headless:=true` — run MuJoCo without the physics GUI window (CI / servers)
- `tcp_offset_xyz:="0 0 0.1358"` — publish `tcp_link` in TF at a fixed offset from `gripper_base`
- `tcp_offset_rpy:="0 0 0"` — rotate the published `tcp_link` frame if needed

MuJoCo keyboard shortcuts in the physics window:
- **Space** — pause / resume
- **→** — single-step while paused
- **Backspace** — reset to home keyframe
- **Ctrl+L** — load model

### With MoveIt2 (MuJoCo + move_group + RViz MotionPlanning)

```bash
ros2 launch agx_arm_mjsim piper_with_gripper_moveit_mjsim.launch.py
```

In RViz use the **MotionPlanning** panel to plan and execute trajectories for the
`arm` and `gripper` planning groups.

Optional arguments:
- `headless:=true` — suppress MuJoCo GUI
- `tcp_offset:="[0.0, 0.0, 0.1358, 0.0, 0.0, 0.0]"` — MoveIt TCP offset `[x, y, z, rx, ry, rz]`; defaults to the midpoint between the gripper jaws

## Architecture

```
piper_with_gripper_moveit_mjsim.launch.py
├── piper_with_gripper_mjsim.launch.py
│   ├── robot_state_publisher      (publishes /robot_description + /tf)
│   ├── mujoco_ros2_control node   (physics engine + controller_manager)
│   │     └── MujocoSystemInterface  loads mjcf/scene.xml, home keyframe
│   ├── joint_state_broadcaster
│   ├── arm_controller             (JointTrajectoryController, position)
│   └── gripper_controller         (JointTrajectoryController, position)
├── move_group                     (MoveIt2, OMPL planning)
└── rviz2                          (MoveIt MotionPlanning plugin)
```

MoveIt reuses `agx_arm_moveit` for SRDF, kinematics, and joint limits; only
the trajectory execution config is overridden for simulation.

## Package structure

```
config/
  initial_positions.yaml    Joint start positions matching the MJCF 'home' keyframe
  ros2_controllers.yaml     Controller manager + JTC config for MuJoCo
  moveit_controllers.yaml   MoveIt controller manager + trajectory execution tolerances
  mjsim.rviz                RViz config for visualization-only launch
launch/
  piper_with_gripper_mjsim.launch.py          MuJoCo + RViz (no MoveIt)
  piper_with_gripper_moveit_mjsim.launch.py   MuJoCo + MoveIt + RViz
mjcf/
  piper.xml     Patched MuJoCo Menagerie model (joint names adapted for ros2_control)
  scene.xml     Top-level scene (includes piper.xml + ground plane + lighting)
  assets/       84 OBJ/STL mesh files (verbatim from MuJoCo Menagerie)
urdf/
  piper_with_gripper_mjsim.urdf.xacro   Top-level robot description for MuJoCo
  piper_mjsim.ros2_control.xacro        ros2_control hardware interfaces
```

## MJCF adaptations

The upstream MuJoCo Menagerie model was modified as follows:

| Change | Reason |
|---|---|
| `joint7` → `gripper_joint1`, `joint8` → `gripper_joint2` | Match SRDF / controller joint names |
| Equality constraint (tendon mimic) removed | Both joints commanded independently via ros2_control for gzsim parity |
| Second `<position>` actuator added for `gripper_joint2` | Required for FollowJointTrajectory dispatch |
| Home keyframe ctrl updated to 8 values | Matches new actuator count |

## Notes

- **Initial positions**: `joint2=1.57`, `joint3=-1.3485` match the MJCF `home` keyframe. All joint start positions are placed slightly inside their URDF limit boundaries to avoid MoveIt `CheckStartStateBounds` rejections.
- **tcp_link**: always present in TF, parented to `gripper_base`, default offset `[0, 0, 0.1358]` (midpoint between gripper jaws). MoveIt plans to this frame.
- **MuJoCo sim services** (provided by `mujoco_ros2_control`):
  ```bash
  ros2 service call /mujoco_ros2_control_node/reset_world    mujoco_ros2_control_msgs/srv/ResetWorld    "{}"
  ```
- **Headless display**: use `headless:=true`; no `DISPLAY` workaround needed.
- **Physics tuning**: inherited from MuJoCo Menagerie (`impratio=10`, `cone=elliptic`, per-joint `kp`/`dampratio`). Adjust in `mjcf/piper.xml` if needed.
