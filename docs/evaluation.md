# Evaluation

> **Results below are not measured yet.** Run the experiments, then fill in the tables.
> Do not submit with the placeholders.

## How to record

```bash
ros2 launch ugv_bringup full_system.launch.py
ros2 bag record -o runs/<experiment> /ekf/odom /true_odom /odom/wheels /imu/data /gnss/fix /cmd_vel /goal_pose
```
Replay/plot with PlotJuggler or `ros2 bag play`. Ground truth is `/true_odom`
(same frame as `/ekf/odom` when spawned at the origin, yaw 0).

## 1. State estimation

Error = `/ekf/odom` − `/true_odom`. Report RMSE of position (m), heading (rad), speed (m/s).

| Scenario | How | Pos RMSE | Heading RMSE | Speed RMSE |
|---|---|---|---|---|
| Stationary | no command, 30 s | TODO | TODO | TODO |
| Straight line | `/cmd_vel` x = 0.2 for 20 s | TODO | TODO | TODO |
| Rotation | z = 0.5 for 20 s | TODO | TODO | TODO |
| Combined | x = 0.2, z = 0.3 | TODO | TODO | TODO |
| Wheel odom only (EKF GPS off) | for comparison | TODO | TODO | TODO |
| GPS dropout (not implemented yet) | stop `true_pose_gps` for 5 s | TODO | TODO | TODO |

## 2. Navigation

| Goal (x, y) | Final position error (m) | Final time (s) | Overshoot / oscillation | Notes |
|---|---|---|---|---|
| (4, 0) | TODO | TODO | TODO | |
| (0, 4) | TODO | TODO | TODO | |
| (-3, -3) | TODO | TODO | TODO | |
| (5, -2) | TODO | TODO | TODO | |

Final error is measured against `/true_odom`, not the EKF estimate.

## 3. Wheel velocity control (after Stage 5)

| Step | Rise time | Settling (2 %) | Overshoot | Steady-state error | Left-right mismatch |
|---|---|---|---|---|---|
| 0 → 0.3 m/s | TODO | TODO | TODO | TODO | TODO |

## 4. Noise / robustness notes

TODO: effect of changing GPS `SIGMA` (and matching `gps_sigma`), IMU noise, GPS dropout.
