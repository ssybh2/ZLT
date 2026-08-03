# soft_drone_manual_controller

这是从 `ssybh2/soft_drone_controller` 中提炼出的 **ROS 2 手动飞行包**。只保留以下链路：

`DJI 遥控器 → 手动摇杆映射 → 四元数姿态外环 → 角速度 PID 内环 → X 型四电机混控 → DSHOT`

已删除 `POSITION`、`HOLD`、`PATH`、动捕位置、目标点和路径发布节点。

## 1. 适用环境

- ROS 2 Humble（其他 ROS 2 版本通常也能运行）
- Python 3、NumPy
- 已经存在的 EtherCAT 驱动和接口消息包
- 默认接口：
  - `custom_msgs/msg/ReadDJIRC`
  - `custom_msgs/msg/WriteDSHOT`
- 默认话题：
  - DJI RC：`/ecat/sn2228293/app1/read`
  - IMU：`/ecat/sn2228293/app2/read`
  - DSHOT：`/ecat/sn2228293/app3/write`

消息类型使用运行时加载，因此你的系统若实际使用 `soft_drone_msgs`，只需要修改 YAML，例如：

```yaml
rc_message_type: soft_drone_msgs/msg/ReadDJIRC
dshot_message_type: soft_drone_msgs/msg/WriteDSHOT
```

## 2. 安装和编译

把整个文件夹放入 ROS 2 工作空间的 `src`：

```bash
cd ~/your_ros2_ws/src
unzip soft_drone_manual_controller.zip
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select soft_drone_manual_controller
source install/setup.bash
```

先确认自定义消息已经被当前终端找到：

```bash
ros2 interface show custom_msgs/msg/ReadDJIRC
ros2 interface show custom_msgs/msg/WriteDSHOT
```

## 3. 第一次启动：必须拆桨并使用 dry-run

```bash
ros2 launch soft_drone_manual_controller manual_controller.launch.py dry_run:=true
```

查看输入和计算结果：

```bash
ros2 topic echo /ecat/sn2228293/app1/read
ros2 topic echo /ecat/sn2228293/app2/read
ros2 topic echo /manual_drone/imu_angle_deg
ros2 topic echo /manual_drone/motor_pwm_us
ros2 topic echo /manual_drone/motor_dshot
ros2 topic echo /manual_drone/armed
```

重新把当前姿态设为零点：

```bash
ros2 service call /manual_drone/reset_attitude_zero std_srvs/srv/Trigger {}
```

节点会拒绝在解锁状态下重置零点。

## 4. DJI 遥控器映射

| DJI 输入 | 手动飞行功能 |
|---|---|
| 左摇杆上下 `left_y` | 总油门，假定范围 `[-1, 1]` |
| 左摇杆左右 `left_x` | Yaw 角速度指令 |
| 右摇杆左右 `right_x` | Roll 姿态角指令 |
| 右摇杆上下 `right_y` | Pitch 姿态角指令，默认取反 |
| 左三段开关位置 `2` | 上锁 |
| 左三段开关位置 `1` 或 `3` | 解锁并保持 MANUAL |
| 右三段开关 | 手动包中不参与模式切换，仅保留读取 |

默认增加两项解锁保护：`left_y <= -0.80` 才允许解锁，并且每次启动或超时上锁后，都必须先把左开关拨到锁定位置，再拨回解锁位置。

## 5. 实机输出

确认以下项目全部正确后，才可关闭 dry-run：

1. 拆桨时逐个核对四路电机通道和旋向。
2. 向右倾斜机身时，控制输出必须产生向左恢复的趋势。
3. 向前倾斜机身时，控制输出必须产生向后恢复的趋势。
4. 核对 IMU 四元数符号、角速度符号和右摇杆 Pitch 符号。
5. 核对 DJI 左开关的实际数值确实是 `1/3=解锁，2=上锁`。
6. 核对油门最低点是 `-1` 附近。

然后执行：

```bash
ros2 launch soft_drone_manual_controller manual_controller.launch.py dry_run:=false
```

也可以直接编辑：

```text
config/manual_controller.yaml
```

## 6. 电机通道顺序

原仓库的内部电机到 DSHOT 通道映射是：

```text
channel1 = motor3
channel2 = motor1
channel3 = motor2
channel4 = motor4
```

对应 YAML：

```yaml
dshot_channel_order: [2, 0, 1, 3]
```

数组使用从 0 开始的内部电机索引。若实机接线不同，只修改此数组，不要先改 PID。

## 7. 与原仓库相比的关键处理

- 保留原仓库最新手动模式的 IMU 符号：四元数 `[w, x, -y, -z]`，角速度 `[x, -y, -z]`。
- 保留四元数误差姿态外环、角速度内环、混控矩阵、平滑和 DSHOT 映射。
- 删除位置控制、动捕世界系对齐、Yaw 保持、路径和目标点节点。
- 自定义消息类型改成运行时参数，解决源代码导入 `custom_msgs`、但 `package.xml` 写成 `soft_drone_msgs` 的不一致。
- 锁定值 `48` 和解锁怠速值 `120` 作为 **原始 DSHOT 数值**发送，不再错误地先当成 `1000–2000 us PWM` 二次转换。
- 增加 RC/IMU 超时自动上锁、低油门解锁限制、关机重复发送锁定值和 dry-run。
- 安全配置默认最大 Roll/Pitch 为 ±30°。原仓库 `_process_stick()` 额外乘以 `3`，会达到 ±90°；需要完全复现时使用 `config/reference_original.yaml`，但仍应先 dry-run。

使用原仓库角度倍率配置：

```bash
ros2 launch soft_drone_manual_controller manual_controller.launch.py \
  config:=$(ros2 pkg prefix soft_drone_manual_controller)/share/soft_drone_manual_controller/config/reference_original.yaml \
  dry_run:=true
```

## 8. 首次调试优先顺序

不要一开始同时调 PID 和符号。正确顺序是：

1. 遥控器通道数值与方向。
2. IMU Roll/Pitch/Yaw 与角速度方向。
3. 四个 DSHOT 通道与电机编号。
4. Roll/Pitch/Yaw 混控正负号。
5. 油门范围和悬停点。
6. 最后再调角速度 PID、姿态时间常数和角度倍率。

> 这是直接驱动旋翼电机的实验控制器。第一次通电、改符号、改混控和改通道顺序时必须拆除螺旋桨，并准备独立断电手段。
