# sn2031674_manual_rc

ROS 2 manual DJI RC -> DShot package for the EtherCAT configuration using slave `sn2031674`.

## Topic mapping

- RC input: `/ecat/sn2031674/app1/read` (`custom_msgs/msg/ReadDJIRC`)
- DShot output: `/ecat/sn2031674/app4/write` (`custom_msgs/msg/WriteDSHOT`)
- Armed diagnostic: `/sn2031674_manual_rc/armed`
- Normalized RC diagnostic: `/sn2031674_manual_rc/manual_axes`

The package follows the ZLT DJI RC mapping exactly:

- `left_y` = throttle
- `left_x` = yaw
- `right_x` = roll
- `right_y` = pitch input; the controller uses `pitch = -right_y`, matching the ZLT manual controller convention
- `right_switch == 3` = ARM
- `right_switch == 1` = DISARM

The EcatV2_Master DJI RC decoder reports stick values in `[-1, 1]`. DShot uses `0` for disarmed, `1..47` for reserved commands, and `48..2047` for thrust.

## Safety behavior

The default configuration is intentionally `dry_run: true`. It reads and validates the RC and reports arming state, but it does **not** publish DShot motor commands.

Arming also requires:

1. RC `online == 1`.
2. The right switch must transition into value `3` (`require_arm_edge: true`).
3. Throttle must be at the bottom (`left_y <= -0.90`).
4. RC timeout (`0.30 s`) immediately disarms.
5. Right switch value `1` immediately disarms.
6. By default, right switch value `2` also disarms as a failsafe.

Remove propellers for the first hardware test.

## Build

From the ROS 2 workspace containing this repository and `custom_msgs`:

```bash
colcon build --symlink-install --packages-select custom_msgs sn2031674_manual_rc
source install/setup.bash
```

If `custom_msgs` is already installed/built in the workspace:

```bash
colcon build --symlink-install --packages-select sn2031674_manual_rc
source install/setup.bash
```

## 1. Verify the RC first

Start EtherCAT bringup, then verify the actual decoded values:

```bash
ros2 topic echo /ecat/sn2031674/app1/read
```

Expected DJI switch encoding is `1 = up`, `3 = middle`, `2 = bottom`.

Run this package in dry-run mode:

```bash
ros2 launch sn2031674_manual_rc manual_rc.launch.py dry_run:=true
```

Check:

```bash
ros2 topic echo /sn2031674_manual_rc/manual_axes
ros2 topic echo /sn2031674_manual_rc/armed
```

`manual_axes.data` is:

```text
[throttle, yaw, roll, pitch, left_switch, right_switch, online]
```

with the same ZLT semantics.

## 2. Propeller-off DShot test

Only after the RC values are confirmed, remove all propellers, put throttle fully down, keep the right switch at `1`, and run:

```bash
ros2 launch sn2031674_manual_rc manual_rc.launch.py dry_run:=false
```

Then move the right switch from `1` to `3`. The node arms only if throttle is low. In the default `throttle_only` mode, moving the left throttle stick upward sends the same DShot value to all four channels. Moving the right switch to `1` sends `0` to all channels immediately.

You can watch the actual DShot command topic with:

```bash
ros2 topic echo /ecat/sn2031674/app4/write
```

## Modes

### `throttle_only` (default)

Only `left_y` commands the motors. All four DShot channels are equal. This is the recommended mode for checking the RC -> EtherCAT -> ESC chain.

### `x_open_loop`

```bash
ros2 launch sn2031674_manual_rc manual_rc.launch.py dry_run:=false mode:=x_open_loop
```

This additionally uses:

- roll = `right_x`
- pitch = `-right_y`
- yaw = `left_x`

through an X-frame open-loop mixer. **It has no IMU attitude/rate feedback and is not a stabilized flight controller.** Use it only for controlled bench experiments unless you deliberately add a proper feedback controller.

## Useful parameters

Edit `config/manual_rc.yaml` to change:

- `dshot_idle` / `dshot_max`
- `arm_throttle_max`
- `rc_timeout_s`
- `dshot_channel_order`
- `open_loop_mix_gain`
- `disarm_on_other_switch_positions`

For a conservative first test, temporarily lower `dshot_max` (for example to `300`) while propellers are removed.
