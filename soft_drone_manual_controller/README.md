# soft_drone_manual_controller

这是 ZLT 的 ROS 2 手动飞行控制包。基础控制链保持为：

`DJI RC → 手动摇杆 → 四元数姿态外环 → 角速度 PID → X-frame mixer → DShot`

当前版本已经接到 **EcatV2 ProductCode 0x06 / 6×HIPNUC IMU** 话题布局，并保留旧代码的主/副 IMU 语义。

## 当前话题映射

| 作用 | 当前话题 | 旧代码对应 |
|---|---|---|
| 主 IMU / 飞控 IMU | `/imu/can1/slot1` | `app2/read` |
| Rod 0 / 原 DIMD 副 IMU | `/imu/can1/slot2` | `app1/read` |
| Rod 1 | `/imu/can1/slot3` | 新增 |
| Rod 2 | `/imu/can2/slot1` | 新增 |
| Rod 3 | `/imu/can2/slot2` | 新增 |
| Rod 4 | `/imu/can2/slot3` | 新增 |
| DJI RC | `/dji_rc` | `app3/read` 的语义 |
| DShot | `/dshot` | `app4/write` 的语义 |

**只有 `/imu/can1/slot1` 参与基础姿态控制、角速度 PID、解锁条件和主 IMU 超时保护。** 其余五个 IMU 只进入结构振动/DIMD 分支，不会替代主 IMU，也不会因为单个 rod IMU 失效而直接让基础飞控上锁。

## 6-IMU DIMD 扩展

入口脚本现在是：

```text
soft_drone_manual_controller/manual_controller_6imu.py
```

原 `manual_controller.py` 保留不动，作为基础飞控和旧双 IMU DIMD 实现。新文件通过继承扩展出：

- 1 路主 IMU；
- 5 路 rod IMU；
- 每路 rod 相对主 IMU 的时间配对、健康状态、对齐角速度和 residual；
- 可配置的 `3×15` DIMD 空间投影矩阵；
- 继续复用原来的 band-pass、phase lead、gain、torque limit、ramp 和 `tau_base + tau_dimd` 控制链。

为了不在更换 EtherCAT 话题的同时改变已经飞过的控制律，默认投影矩阵只选择 **rod0 = `/imu/can1/slot2`**。因此默认 DIMD 控制信号仍等价于旧代码的 `app1 - app2` 路径；其余四个 IMU 已经订阅、时间配对并发布诊断，后续拿到实机模态数据后再修改 `dimd_rod_projection_matrix_flat` 即可真正加入多点模态投影。

每个 rod 的诊断话题为：

```text
/manual_drone/dimd/rod0/gyro_aligned_rad_s
/manual_drone/dimd/rod0/residual_rad_s
/manual_drone/dimd/rod0/healthy
...
/manual_drone/dimd/rod4/gyro_aligned_rad_s
/manual_drone/dimd/rod4/residual_rad_s
/manual_drone/dimd/rod4/healthy
```

## EtherCAT 配置

ZLT 的：

```text
src/soem_bringup/config/config.yaml
```

已经更新为 ProductCode `0x06` 的 8-task 布局：前 6 个 task 为 IMU，第 7 个为 DJI RC，第 8 个为 DShot。当前 ZLT 从站序列号仍使用 `sn2883658`。

`src/EcatV2_Master` 子模块也改为：

```text
https://github.com/ssybh2/EcatV2_Master.git
feature/6imu-rc-dshot-pdo-v006
```

并固定到当前 ProductCode 0x06 分支版本，避免拉回旧 upstream 代码。

## 拉取与编译

```bash
git pull
git submodule sync --recursive
git submodule update --init --recursive

colcon build --symlink-install
source install/setup.bash
```

确认话题：

```bash
ros2 topic list | grep -E 'imu/can|dji_rc|dshot'
ros2 topic echo /imu/can1/slot1
ros2 topic echo /imu/can1/slot2
ros2 topic echo /dji_rc
```

第一次验证建议拆桨并使用 dry-run：

```bash
ros2 launch soft_drone_manual_controller manual_controller.launch.py dry_run:=true
```

然后检查：

```bash
ros2 topic echo /manual_drone/imu_angle_deg
ros2 topic echo /manual_drone/dimd/rod0/residual_rad_s
ros2 topic echo /manual_drone/dimd/rod1/residual_rad_s
ros2 topic echo /manual_drone/motor_pwm_us
ros2 topic echo /manual_drone/armed
```

## 坐标与安装

现有代码继续保留原来的 HIPNUC 转换：

```text
quaternion: [w, x, y, z] -> [w, x, -y, -z]
gyro:       [x, y, z]    -> [x, -y, -z]
```

即原始 FLU → 控制器 FRD。新增四个 rod IMU 的固定空间校准矩阵默认都是单位阵；如果实际安装方向不同，先在 `config/manual_controller.yaml` 中修改 `additional_rod_to_primary_rotation_matrices_flat`，不要直接改 PID 或 DIMD gain。

## 安全说明

第一次更换 ProductCode、PDO、IMU 符号、DShot 通道、混控符号或 DIMD 投影时请拆除螺旋桨，并准备独立断电方式。先验证 6 路 IMU、RC、DShot 和主 IMU 基础姿态控制，再逐步修改多点 DIMD 投影权重。
