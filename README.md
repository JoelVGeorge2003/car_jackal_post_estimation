# Jackal to Car Navigation using pose with A*

A ROS 2 simulation in which a Clearpath Jackal navigates around a parked Toyota Prius and aligns behind it. The implementation combines a vehicle-relative goal, a 2D A* planner and a waypoint-following controller.

[Working video](https://drive.google.com/file/d/1X7kNGQKWujHTdD4wnx8rM7u4VUJG2K8O/view)

- [jvg_astar_det.py](jvg_astar_det.py): Python implementation, including pose retrieval, goal calculation, A* planning and velocity control.

## Environment

| Component | Version|
| --- | --- |
| OS | Ubuntu 24.04 ARM64 |
| ROS | ROS 2 Jazzy |
| Simulator | Gazebo Harmonic |
| Robot | Clearpath Jackal J100 |
| Language | Python 3 |
| World | jackal_lab |
| Jackal model | j100_0000/robot |
| Car model | prius_01 |

## Implementation

### 1. Car and robot pose

The script retrieves the Prius and Jackal poses directly from Gazebo:

```bash
gz model -m prius_01 -p
gz model -m 'j100_0000/robot' -p
```

It parses the model position and orientation and uses world-frame x, y and yaw for navigation. The car pose is read at startup, and the Jackal pose is updated during movement. **These are simulator ground-truth poses. This version does not estimate the car pose from images or LiDAR**, and it does not automatically detect or identify an unknown vehicle. The car model name, dimensions and forward-axis offset are supplied in the code.

### Rear-approach

The vehicle is approximated as a 4.5 m × 1.8 m rectangle. The script applies CAR_FRONT_AXIS_DEG to the model yaw to obtain the assumed front direction.

The target is placed behind the car center at a distance of half the car length plus REAR_CLEARANCE. The final Jackal heading matches the vehicle's forward direction, so it faces toward the rear of the car.

The uploaded script uses a −90° front-axis offset and a 1.0 m rear clearance. Verify the offset against the actual Prius mesh orientation. The target refers to the robot pose origin, rather than a measured bumper-to-bumper gap.

### A* does these
- it built a 2D grid with 0.5 m resolution.
- Represented the car as an oriented rectangular obstacle, inflated by 0.4 m.
- Explored eight neighboring 2d cells with Euclidean movement costs and heuristic.
- Prevent diagonal corner-cutting through blocked neighboring cells.
- Reconstruct the route and simplify it using sampled collision checks along straight segments.
- Only the modeled car footprint is included in the planner; other obstacles are not mapped.

### Motion control

The controller follows the path through TURN, DRIVE and FINAL_ALIGN states. It first turns toward a waypoint, then drives while correcting heading error, and finally aligns to the target yaw.

Commands use geometry_msgs/msg/TwistStamped on:

```text
/j100_0000/platform/cmd_vel
```

The ROS timer publishes at 10 Hz. Maximum configured speeds are 0.6 m/s linear and 0.8 rad/s angular. Final position and yaw tolerances are 0.15 m and 0.06 rad. These values are controller settings, not measured accuracy claims.

The script includes pose-query retries, a stale-command stop, a navigation timeout and a no-movement check.

## What sensors does Jackal use?

The navigation script does not subscribe to camera, LiDAR, IMU, GPS or wheel-odometry topics. Its feedback comes from Gazebo model poses.

The Clearpath model can be configured separately, but the original robot.yaml is not included here, so this README does not claim a particular active sensor configuration. The navigation demonstrated here does not rely on sensor-based vehicle perception or robot localization.

## Setup and execution

With ROS 2 Jazzy installed and the appropriate package repositories configured:

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-clearpath-simulator
```

The original setup used the following local files:

- ~/clearpath/robot.yaml
- ~/jackal_sim/worlds/jackal_lab.sdf
- ~/jackal_sim/models/prius_hybrid/prius_static.sdf
- ~/jackal_sim/models/prius_hybrid/meshes/Hybrid.obj

These robot/world/model assets are prerequisites and are not bundled in this two-file source upload. The custom world defines its ground plane and directional light inline in SDF/XML, avoiding unresolved external ground_plane and sun model references.

The Prius model reference is [OSRF car_demo](https://github.com/osrf/car_demo). Its original ROS Kinetic / Gazebo Classic launch stack differs from this Jazzy / Harmonic setup; the demonstration used a locally adapted static Prius SDF.

### Terminal 1 — Gazebo

```bash
source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH="$HOME/jackal_sim/models:$GZ_SIM_RESOURCE_PATH"
gz sim -r ~/jackal_sim/worlds/jackal_lab.sdf
```

### Terminal 2 — Jackal

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch clearpath_gz robot_spawn.launch.py \
  world:=jackal_lab setup_path:=$HOME/clearpath \
  x:=0.0 y:=0.0 z:=0.3 yaw:=0.0
```

### Terminal 3 — Prius

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch ros_gz_sim gz_spawn_model.launch.py \
  world:=jackal_lab \
  file:="$HOME/jackal_sim/models/prius_hybrid/prius_static.sdf" \
  entity_name:=prius_01 x:=5.0 y:=2.0 z:=0.1 yaw:=0.0
```

### Terminal 4 — Navigation

From the folder containing the uploaded Python file:

```bash
source /opt/ros/jazzy/setup.bash
python3 jvg_astar_det.py
```

### Controller checks

During development, the usual /j100_0000/cmd_vel route did not produce movement. Inspecting the controller and topic connections identified /j100_0000/platform/cmd_vel as the working direct input:

```bash
ros2 topic type /j100_0000/platform/cmd_vel
ros2 control list_controllers -c /j100_0000/controller_manager
ros2 node info /j100_0000/platform_velocity_controller
```

If ros2 control is unavailable, install ros-jazzy-ros2controlcli. Direct platform commands bypass the usual velocity multiplexer; this implementation is for simulation.

