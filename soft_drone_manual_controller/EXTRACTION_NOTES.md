# Manual-mode extraction notes

Reference repository: `ssybh2/soft_drone_controller`

Reference branch/commit inspected: `main` / `ed4e8068afefa85c59edba9e101f19a2912e5254` (`MANUAL+HOLD+PATH`).

## Retained control chain

- `ReadDJIRC` fields: `left_y`, `left_x`, `right_x`, `right_y`, `left_switch`, `right_switch`
- `Imu.orientation` and `Imu.angular_velocity`
- IMU quaternion sign correction `[w, x, -y, -z]`
- Gyroscope sign correction `[x, -y, -z]`
- Relative attitude zero from the first IMU quaternion
- Manual RC dead zones and right-Y inversion
- Quaternion attitude-error outer loop
- Roll, pitch and yaw angular-rate PID loops
- X-frame four-motor mixer
- Motor spread compression and first-order output smoothing
- PWM-to-DSHOT conversion
- Source channel permutation `[2, 0, 1, 3]`
- RC/IMU timeout lock behavior

## Removed

- POSITION mode
- HOLD/PATH switching
- Motion-capture pose subscription and world-frame alignment
- Position attitude command subscription
- Yaw setpoint subscription and yaw hold
- Position controller, target sender and path command nodes

## Intentional corrections and safeguards

1. The source imports `custom_msgs`, while its `package.xml` declares `soft_drone_msgs`. The extracted package loads both RC and DSHOT types by parameter at runtime.
2. The source passes raw lock/unlock DSHOT values through a PWM conversion function. The extracted package has a separate raw-DSHOT path.
3. Arming requires low throttle by default.
4. Startup and timeout recovery require a lock-switch cycle before arming.
5. `dry_run` defaults to true.
6. The safe YAML uses a ±30° manual attitude limit. `reference_original.yaml` retains the source multiplier of 3.0 (±90°).
