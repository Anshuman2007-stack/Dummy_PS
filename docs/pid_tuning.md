# PID tuning methodology

## What is tuned today

Only the **navigation** PIDs (distance → `v`, heading error → `ω`) in `navigation_pid_node.py`.
Per-wheel velocity PID (Stage 5) is not implemented yet, so there is no wheel-level tuning to report.

## Suggested procedure for the navigation gains

1. Set `Ki = Kd = 0`. Raise `Kp` until the response is fast but not oscillating.
2. Add `Kd` to damp overshoot (heading: 0.2, distance: 0.1).
3. Add `Ki` only if there is a persistent steady-state error (currently 0 for both loops).
4. Verify on several goals in different quadrants; watch for oscillation near the goal and
   widen `goal_tol` / reduce gains if it appears.

Test with:
```bash
ros2 launch ugv_bringup full_system.launch.py
# goal window:  4 0   then   0 4   then   -3 -3   then   5 -2
ros2 topic echo /cmd_vel
```

## Plan for the per-wheel PID (Stage 5)

1. Add a node that subscribes to `/cmd_vel`, applies the kinematics from `math.md` and runs one PID per wheel
   against measured wheel velocity (from joint states or encoder odometry).
2. Tune one wheel with the other held still: **Ziegler–Nichols** (`Ki = Kd = 0`, increase `Kp` to the
   ultimate gain `Ku` giving sustained oscillation, measure period `Tu`; PID: `Kp = 0.6 Ku`,
   `Ki = 2 Kp/Tu`, `Kd = Kp·Tu/8`), then reduce by hand for less overshoot.
3. Apply the **same gains** to all wheels (avoids left/right desync) and verify encoder scaling.
4. Record step responses (e.g. 0 → 0.3 m/s) and report rise time, settling time (2 %), overshoot and
   steady-state error in `evaluation.md`.
