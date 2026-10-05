# mobilerobot_toxic

Mecanum-wheel mobile robot (ROS 2 Jazzy) with a Gazebo Harmonic simulation,
SLAM mapping, and Nav2 autonomous navigation. Controlled with a PS5 controller.

## Features
- SolidWorks-exported URDF with RViz visualization
- Gazebo Harmonic simulation with a custom 10 x 8 m map (`worlds/custom_map.sdf`)
- Mecanum drive (Gazebo MecanumDrive plugin), 360 deg lidar (`/scan`), IMU (`/imu`)
- SLAM with `slam_toolbox` (mapping in RViz)
- Autonomous navigation with Nav2 + AMCL on a saved map
- PS5 (DualSense) teleop with a deadman button; also works as a manual override during navigation

## Requirements
Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic.

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-slam-toolbox ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup ros-jazzy-joy ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-robot-state-publisher ros-jazzy-rviz2 xterm
```

## Build
```bash
cd ~/ros2_ws/src
git clone https://github.com/thanratanak/mobilerobot_toxic.git
cd ~/ros2_ws
colcon build --packages-select mobilerobot_toxic
source install/setup.bash
export ROS_DOMAIN_ID=0      # must be 0-101
```

## Usage

### 1. SLAM: build a map
```bash
ros2 launch mobilerobot_toxic gazebo_slam.launch.py enable_joy:=true
```
Hold **R1** and drive slowly around the room (turn slowly and finish where you started).
Save the map:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```
A ready-made map for the default world is in `maps/`.

### 2. Navigation: drive to goals
```bash
ros2 launch mobilerobot_toxic gazebo_nav.launch.py enable_joy:=true
# a different map:  map:=/home/<user>/my_map.yaml
```
In RViz: **2D Pose Estimate** (if the robot is misaligned), then **Nav2 Goal**.

### Launch arguments
| Argument | Default | Meaning |
|---|---|---|
| `world` | `custom_map.sdf` | World file name in `worlds/` or an absolute path |
| `gui` | `true` | Show the Gazebo window |
| `enable_joy` | `false` | PS5 controller teleop |
| `enable_teleop` | `false` | Keyboard teleop (SLAM launch only) |
| `slam` / `rviz` | `true` | SLAM launch only |
| `use_rviz` | `true` | Navigation launch only |
| `map` | `~/my_map.yaml` | Navigation launch only |
| `x`, `y`, `yaw` | `0` | Spawn pose |

If you change the spawn pose for navigation, also change `initial_pose` in `config/nav2_params.yaml`.

## PS5 controller
| Input | Action |
|---|---|
| Hold **R1** | Enable (deadman) |
| Left stick | Forward/back and strafe |
| Right stick (left/right) | Rotate |
| Press **R3** | Turbo |

The node publishes `/cmd_vel` only while R1 is held, so holding R1 takes over from Nav2.
Button and axis numbers can be changed in `scripts/ps5_teleop.py`
(check them with `ros2 topic echo /joy`).

## Package layout
```
config/     nav2_params.yaml, slam_toolbox_params.yaml
launch/     gazebo_slam.launch.py, gazebo_nav.launch.py, RViz and fake-controller launch files
maps/       saved map (my_map.pgm / my_map.yaml)
meshes/     original STL meshes (used by RViz)
meshes_gz/  lightweight meshes (used by Gazebo)
rviz/       RViz configs
scripts/    ps5_teleop.py, fake controller and fake lidar scripts
urdf/       RobotNavigation.urdf
worlds/     custom_map.sdf
```

The Gazebo launch builds a Gazebo-specific copy of the URDF at start-up (simple
collision shapes, mecanum plugin, lidar, IMU). `urdf/RobotNavigation.urdf` is not modified.

## Notes
- The saved map must match the world. If you edit the world, map it again.
- `ROS_DOMAIN_ID` must be 0-101; larger values make every ROS node crash on start.
