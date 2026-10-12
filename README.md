# UGV EKF Localisation and Target Navigation (ROS 2 + Gazebo)

Simulation of a differential-drive UGV that estimates its own state by fusing
**wheel odometry + IMU + GPS** in an **Extended Kalman Filter**, then drives to a
target `(x, y)` typed in by the user.

```
 Gazebo ──► /odom/wheels ─┐
 Gazebo ──► /imu/data ────┼─► EKF ──► /ekf/odom ──► Navigation ──► /cmd_vel ──► Gazebo DiffDrive
 true_pose_gps ► /gnss/fix┘  (ekf_node)             (navigation_pid_node)
        ▲                                                  ▲
   /true_odom (ground truth)                       /goal_pose ◄── goal_pose_node (keyboard)
```

> Status against the problem statement, and what is still missing, is listed in
> [Project status](#project-status). Please read it before submitting.

---

## 1. Repository layout

```
ugv-ekf-navigation/
├── src/
│   ├── my_bot_sim/        Stage 1  Gazebo world, robot model, bridge, true_pose_gps (GPS source)
│   ├── ugv_nav/           Stage 2+3  ekf_node, navigation_pid_node, goal_pose_node, params
│   └── ugv_bringup/       Stage 6  one launch file that starts everything
├── scripts/               setup.sh · check_system.sh · kill_sim.sh
├── docs/                  architecture · maths · PID tuning · evaluation
├── README.md  LICENSE  .gitignore
```

| Stage | Package / file | Run on its own with |
|---|---|---|
| 1 Sensor interfaces | `my_bot_sim` | `ros2 launch my_bot_sim sim.launch.py` |
| 2 EKF fusion | `ugv_nav/ekf_node.py` | `ros2 launch ugv_nav ekf.launch.py` |
| 3 Target navigation | `ugv_nav/navigation_pid_node.py` | `ros2 launch ugv_nav navigation.launch.py` |
| – Goal input (keyboard) | `ugv_nav/goal_pose_node.py` | `ros2 run ugv_nav goal_pose_node` |
| 4 Kinematics, 5 wheel PID | *not implemented yet* – see [Project status](#project-status) | |
| 6 End-to-end | `ugv_bringup` | `ros2 launch ugv_bringup full_system.launch.py` |

## 2. Requirements

* Ubuntu 22.04 + **ROS 2 Humble** + **Gazebo Fortress** (Ignition Gazebo 6). The SDF files use the
  Fortress plugin names (`ignition-gazebo-*`) and the bridge uses `ignition.msgs.*` types.
  This will **not** load on Garden/Harmonic without renaming them.
* `ros-humble-ros-gz` (provides `ros_gz_sim`, `ros_gz_bridge`; pulls in Fortress), `python3-numpy`,
  `xterm` (for the goal-input window), `colcon`, `rosdep`.

## 3. Build (step by step)

```bash
# 1. Install ROS 2 Humble: https://docs.ros.org/en/humble/Installation.html
sudo apt install ros-humble-desktop ros-humble-ros-gz
ign gazebo --version        # should print 6.x (Fortress). If 'ign' is missing, install ignition-fortress
                            # from the OSRF apt repo: https://gazebosim.org/docs/fortress/install_ubuntu

# 2. Get the code
git clone https://github.com/<your-user>/ugv-ekf-navigation.git
cd ugv-ekf-navigation

# 3. Install dependencies and build (or run ./scripts/setup.sh)
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt install xterm
colcon build --symlink-install

# 4. Source the workspace (needed in EVERY new terminal)
source install/setup.bash
```

Verify the build: `ros2 pkg list | grep -E "my_bot_sim|ugv_nav|ugv_bringup"` should print all three.

## 4. Run the whole project (one command)

```bash
source install/setup.bash
ros2 launch ugv_bringup full_system.launch.py
```

This starts: Gazebo + world + robot, the ROS↔Gazebo bridge, `true_pose_gps`, `ekf_node`,
`navigation_pid_node` and `goal_pose_node` (in an xterm window).

Launch arguments:

| Argument | Default | Meaning |
|---|---|---|
| `headless` | `false` | `true` = Gazebo server only, no GUI |
| `goal_input` | `auto` | `auto` / `xterm` / `gnome-terminal` / `none`. `none` = start `goal_pose_node` yourself |

### Send the robot to a target

1. **Wait ~5 s without moving the robot** until the `ekf_node` log prints
   `GPS origin set. Ready for goals.` (it averages the first 20 GPS fixes).
2. In the goal window type `x y` in metres and press Enter, e.g. `4 5`.
   Type `pos` to see the current estimate, `q` to quit. A new goal replaces the old one.
3. Goals are in the EKF `odom` frame, which starts at `(0, 0)` where the robot spawns
   (= the world origin).

No xterm window? Use a second terminal:

```bash
source install/setup.bash
ros2 run ugv_nav goal_pose_node
```

Or publish a goal directly:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: odom}, pose: {position: {x: 4.0, y: 5.0}, orientation: {w: 1.0}}}"
```

## 5. Run stage by stage (for testing each block independently)

Use a separate terminal for each command (source the workspace in each).

```bash
# Stage 1 - sensors only: Gazebo + robot + bridge + GPS
ros2 launch my_bot_sim sim.launch.py
#   check:  ros2 topic hz /odom/wheels /imu/data /gnss/fix
#   drive:  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"

# Stage 2 - add the EKF (robot must stay still for the first ~5 s)
ros2 launch ugv_nav ekf.launch.py
#   check:  ros2 topic echo /ekf/odom --field pose.pose.position
#   compare against /true_odom (see Debugging)

# Stage 3 - add the navigation controller
ros2 launch ugv_nav navigation.launch.py
ros2 run ugv_nav goal_pose_node          # then type:  3 2
```

Or the last two together: `ros2 launch ugv_nav nav_stack.launch.py`.

## 6. Debugging cheat-sheet

### Is everything alive?

```bash
./scripts/check_system.sh                 # nodes + topic data + rates, PASS/FAIL
ros2 node list                            # expect: true_pose_gps ekf_node navigation_pid_node goal_input (+ ros_gz_bridge)
ros2 topic list -t
ros2 run rqt_graph rqt_graph              # visual node/topic graph
```

### Sensors (Stage 1)

```bash
ign topic -l                              # topics inside Gazebo: /imu /navsat /model/my_bot/odometry /true_odom /cmd_vel
ign topic -e -t /imu -n 1                 # one raw Gazebo IMU message
ros2 topic hz /odom/wheels                # ~50 Hz
ros2 topic hz /imu/data                   # ~100 Hz
ros2 topic hz /gnss/fix                   # ~5 Hz
ros2 topic echo /odom/wheels --once
ros2 topic echo /imu/data --field angular_velocity --once
ros2 topic echo /gnss/fix --field latitude --once
```

Drive manually and confirm the sensors react (`Ctrl-C`, then send zeros to stop):

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"   # straight
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"   # spin
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"                                     # stop
# keyboard alternative: sudo apt install ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### EKF (Stage 2): estimate vs ground truth

```bash
ros2 topic echo /ekf/odom  --field pose.pose.position --once
ros2 topic echo /true_odom --field pose.pose.position --once     # same frame when spawned at (0,0,yaw 0)
ros2 topic echo /ekf/odom  --field pose.covariance                # shrinking P = healthy filter
ros2 topic hz /ekf/odom                                           # ~50 Hz
```

Plot them together: `ros2 run plotjuggler plotjuggler` (install `ros-humble-plotjuggler-ros`)
or `ros2 run rqt_plot rqt_plot /ekf/odom/pose/pose/position/x /true_odom/pose/pose/position/x`.

### Navigation (Stage 3)

```bash
ros2 topic echo /goal_pose --once
ros2 topic echo /cmd_vel                  # should go to 0 when "Goal reached." is logged
ros2 topic info /goal_pose -v             # navigation_pid_node must appear as a subscriber
```

The controller logs its state every 0.5 s: `ROTATE | DRIVE | ALIGN` with distance, heading error and command.

### Record a run (for the evaluation section)

```bash
ros2 bag record -o run_01 /ekf/odom /true_odom /odom/wheels /imu/data /gnss/fix /cmd_vel /goal_pose
ros2 bag info run_01
```

### Parameters

Edit `src/ugv_nav/config/ugv_nav.yaml` and relaunch (with `--symlink-install` no rebuild is needed).
The nodes read their parameters once at start-up, so `ros2 param set` has no effect on a running node.

```bash
ros2 param list /ekf_node
ros2 param get /navigation_pid_node goal_tol
```

### Common problems

| Symptom | Likely cause / fix |
|---|---|
| `Package 'my_bot_sim' not found` | Forgot `source install/setup.bash` in this terminal. |
| `executable 'true_pose_gps.py' not found` | Stale build. `rm -rf build install log && colcon build --symlink-install`. |
| Gazebo opens, no robot | Check the `create` node output; world/model paths come from the installed package share dir. |
| `/odom/wheels`, `/imu/data` empty | Bridge not running or Gazebo paused (world is started with `-r`). Compare `ign topic -l` with `ros2 topic list`. |
| `/true_odom` missing, GPS silent | `true_pose_gps` waits for `/true_odom`. Check the bridge line and the plugin name in `model.sdf`; the error text is in the bridge/Gazebo log. |
| EKF never prints `GPS origin set` | No `/gnss/fix` (see above). It needs 20 fixes (~4 s at 5 Hz). |
| Robot ignores goals | `ros2 topic info /goal_pose -v` – is the controller subscribed? Is `/ekf/odom` publishing? |
| Estimate jumps or drifts | Make sure the robot was still during GPS-origin averaging; check `gps_sigma` equals the noise in `true_pose_gps.py`. |
| Robot oscillates near goal | Raise `goal_tol` or lower the gains in `navigation_pid_node.py`. |
| Goal window did not open | Install xterm, or `goal_input:=none` and run `ros2 run ugv_nav goal_pose_node` in another terminal. |
| `Failed to load system plugin [gz-sim-...]` or a robot that does not move | Plugin names from Garden/Harmonic. In Fortress they must be `ignition-gazebo-*-system` / `ignition::gazebo::systems::*` (already done in this repo's SDF files). |
| Bridge error `unsupported/unknown type ... gz.msgs` | On Fortress the bridge types are `ignition.msgs.*`. |
| `ign: command not found` | Fortress not installed: `sudo apt install ros-humble-ros-gz` or install `ignition-fortress` from the OSRF repo. |
| Stale processes / Gazebo won't restart | `./scripts/kill_sim.sh` |
| Weird behaviour after editing files | `colcon build --symlink-install --packages-select <pkg>` then re-source. |

## 7. Topics and frames

| Topic | Type | Producer | Consumer |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | `navigation_pid_node` | Gazebo DiffDrive (via bridge) |
| `/odom/wheels` | `nav_msgs/Odometry` | Gazebo DiffDrive (encoder odometry) | `ekf_node` |
| `/imu/data` | `sensor_msgs/Imu` | Gazebo IMU | `ekf_node` |
| `/gnss/fix` | `sensor_msgs/NavSatFix` | `true_pose_gps` | `ekf_node` |
| `/true_odom` | `nav_msgs/Odometry` | Gazebo OdometryPublisher | `true_pose_gps`, evaluation only |
| `/ekf/odom` | `nav_msgs/Odometry` | `ekf_node` | `navigation_pid_node`, `goal_pose_node` |
| `/goal_pose` | `geometry_msgs/PoseStamped` | `goal_pose_node` | `navigation_pid_node` |

Robot: wheel radius 0.1 m, wheel separation 0.4 m, limits 0.5 m/s and 1.5 rad/s.
GPS: 5 Hz, σ = 0.3 m, origin 22.62° N, 75.69° E (must match `LAT0/LON0` in `true_pose_gps.py` and `my_world.sdf`).

More detail: [architecture](docs/architecture.md), [maths](docs/math.md),
[PID tuning](docs/pid_tuning.md), [evaluation](docs/evaluation.md).

## 8. Project status

Checked against the problem statement (Section 1.4):

| # | Stage | State |
|---|---|---|
| 1 | Sensor interfaces | Done. Wheel odometry comes from Gazebo's DiffDrive plugin, not computed from raw encoder ticks. |
| 2 | EKF fusion | Done: state `(x, y, θ, v, ω)`, wheel + gyro + IMU yaw + GPS with chi-square gating. |
| 3 | Target navigation | Done: rotate → drive state machine with PID on heading and distance. |
| 4 | Diff-drive kinematics (4 wheels) | **Missing.** `/cmd_vel` goes straight to Gazebo's DiffDrive plugin. |
| 5 | Per-wheel PID with anti-windup | **Missing.** The only PID is the heading/distance one in the navigation node. |
| 6 | Integration + TF | Launch files done. **No `odom → base_link` TF is published yet** (the EKF only publishes `Odometry`). |

Also open: the model has two driven wheels plus casters (problem statement says four-wheel);
GPS dropout, wheel slip and actuator saturation scenarios are not yet exercised;
the evaluation results in `docs/evaluation.md` still have to be measured.

## 9. License

MIT, see [LICENSE](LICENSE).
