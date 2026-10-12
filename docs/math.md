# Mathematical formulation

## 1. Differential-drive kinematics

With wheel radius `r = 0.1 m` and wheel separation `L = 0.4 m`, for body velocities `(v, ω)`:

```
ω_left  = (v - ω·L/2) / r          ω_right = (v + ω·L/2) / r
v = r·(ω_right + ω_left)/2         ω = r·(ω_right - ω_left)/L
```

Currently this conversion happens **inside Gazebo's DiffDrive plugin** (parameters in
`model.sdf`); there is no separate kinematics node. For a four-wheel skid-steer model the
left pair uses `ω_left` and the right pair `ω_right`.

## 2. Wheel odometry

The DiffDrive plugin integrates the encoder angles and publishes `/model/my_bot/odometry`
(bridged to `/odom/wheels`). The EKF uses only its **twist** `(v, ω)`, not its drifting pose.

## 3. EKF (`ekf_node.py`)

State `x = [x, y, θ, v, ω]ᵀ`.

**Prediction** (every 20 ms, unicycle model, `v` and `ω` constant over `dt`):

```
x ← x + v·cosθ·dt        y ← y + v·sinθ·dt        θ ← wrap(θ + ω·dt)
P ← F P Fᵀ + Q·dt
```
```
F = I with  F[0,2] = -v·sinθ·dt   F[0,3] = cosθ·dt
            F[1,2] =  v·cosθ·dt   F[1,3] = sinθ·dt   F[2,4] = dt
Q = diag(1e-3, 1e-3, 1e-3, 0.1, 0.1)
```

**Updates** (applied whenever a message arrives, hence multi-rate / asynchronous):

| Sensor | Measurement `z` | `H` | `R` |
|---|---|---|---|
| Wheel odometry (50 Hz) | `[v, ω]` | picks `v, ω` | `diag(0.01, 0.02)` |
| IMU gyro (100 Hz) | `ω` (angular_velocity.z) | picks `ω` | `0.01` |
| IMU orientation (100 Hz, optional) | `θ` = yaw + fixed offset, angle-wrapped residual | picks `θ` | `0.01` |
| GPS (5 Hz) | `[x, y]` = start position + local ENU of the fix | picks `x, y` | `σ²·I`, σ = 0.3 m |

Update equations (Joseph form for numerical stability):

```
y = z - H x            S = H P Hᵀ + R           K = P Hᵀ S⁻¹
x ← x + K y            P ← (I-KH) P (I-KH)ᵀ + K R Kᵀ
```

**Outlier gating:** a GPS fix is rejected if `yᵀ S⁻¹ y > 9.21` (χ², 2 dof, 99 %). After 20 consecutive
rejections the filter is assumed lost and the next fix is accepted unconditionally.

**GPS to local metres:** `east = R_E·(lon - lon₀)·cos(lat₀)`, `north = R_E·(lat - lat₀)`, with
`(lat₀, lon₀)` the mean of the first 20 fixes.

## 4. Target navigation (`navigation_pid_node.py`)

Given goal `(x_g, y_g)` and estimate `(x, y, θ)`:

```
d = hypot(x_g - x, y_g - y)        e = wrap(atan2(y_g - y, x_g - x) - θ)
```

State machine (20 Hz):

* **ROTATE** – `v = 0`, `ω = PID_ang(e)`; go to DRIVE when `|e| < rotate_done` (0.08 rad).
* **DRIVE** – `v = PID_lin(d)·max(0, cos e)`; `ω = PID_ang(e)` while `d > steer_freeze` (0.15 m).
  Back to ROTATE if `|e| > rotate_again` (0.6 rad) and `d > steer_freeze`. If the robot passes the
  goal (`d ≤ steer_freeze`, `|e| > π/2`) it stops when `d < 2·goal_tol`, else turns around.
* **Done** when `d < goal_tol` (0.10 m). Optional **ALIGN** to the goal yaw (`align_final`).

PID gains: linear `Kp 0.8, Ki 0, Kd 0.1`, output clamp `max_linear`; angular `Kp 2.0, Ki 0, Kd 0.2`,
output clamp `max_angular`. The integrator is conditionally frozen while saturated, clamped, and reset when
the error crosses zero (anti-windup). Steering is frozen near the goal because heading is noisy at small
distances. The `cos e` factor slows the robot while it is mis-aligned.
