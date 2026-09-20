# sn2031674_manual_rc：sn2883650 四任务 RC → DShot

> **说明：** ROS 2 包名和节点名仍叫 `sn2031674_manual_rc`（兼容原来的 launch 命令），**实际 EtherCAT 从站序列号已改为 `sn2883650`**。请不要将旧包名误认为实际从站号。

## 与当前四任务 EtherCAT YAML 对应的话题

| 应用 | 作用 | ROS 2 话题 | 消息类型 / 使用方 |
| --- | --- | --- | --- |
| app1 | DJI 遥控器 | `/ecat/sn2883650/app1/read` | `custom_msgs/msg/ReadDJIRC`，本包订阅 |
| app2 | CAN1 IMU | `/ecat/sn2883650/app2/read` | `sensor_msgs/msg/Imu`，本包**不订阅** |
| app3 | CAN2 IMU | `/ecat/sn2883650/app3/read` | `sensor_msgs/msg/Imu`，本包**不订阅** |
| app4 | DShot | `/ecat/sn2883650/app4/write` | `custom_msgs/msg/WriteDSHOT`，本包发布 |

对应的配置保存在 `src/soem_bringup/config/config.yaml`：从站 `sn2883650`、4 个任务、RC PDO 读偏移 0，两个 IMU 的 PDO 读偏移 19、40，DShot PDO 写偏移 0。固件中的 PDO 布局、从站序列号、DShot 端口 `sdowrite_dshot_id: 1` 必须与实际硬件一致；**ROS 话题名称一致不意味着固件/PDO 已经匹配。**

本包是**仅由遥控器控制的开环 DShot 测试节点**，不使用两个 IMU 做姿态/角速度反馈，不是可直接用于飞行的稳定控制器。本仓库另一包 `soft_drone_manual_controller` 的默认配置仍针对原有 **6 IMU** 布局，**不要直接用它的默认 launch 连接这里的 4-task 从站**，也不要同时启动两个向同一 DShot 话题发布的控制节点。

## 安全逻辑

- 默认 `dry_run: true`，只读遥控器、打印计算出的电机命令，**不发布 DShot**。
- 遥控器 `online == 1` 且右拨杆从其他位置切换到 `3`、左油门 `left_y <= -0.90` 才可解锁；右拨杆为 `1` 或其他位置会锁定。
- 遥控器超时 0.30 秒会锁定，发送 DShot 零值（`dry_run: false` 时）。
- `throttle_only` 默认把相同 DShot 值发给 4 个通道；`x_open_loop` 不含姿态反馈，不能替代飞控。

**首次通电测试必须拆除螺旋桨，避免在有动力的状态下操作接线。** 保持能够独立切断动力电源。

## 1. 拉取指定分支并编译 ROS 2

以下假设 `~/ZLT` 是你的 **ROS 2 工作空间根目录**；若实际路径不同，替换成你自己的目录。

新克隆：

```bash
cd ~
git clone -b sn2031674-manual-rc-dshot --recurse-submodules \
    https://github.com/ssybh2/ZLT.git
cd ~/ZLT
```

已经克隆过则：

```bash
cd ~/ZLT
git switch sn2031674-manual-rc-dshot
git pull --ff-only
git submodule update --init --recursive
```

确认 ROS 2 环境、EtherCAT backend 和消息包依赖已经准备好，再编译（如果你已经在别的工作空间编译/安装了 `custom_msgs`，先 source 对应 `install/setup.bash`）：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to \
    soem_bringup sn2031674_manual_rc
source install/setup.bash
```

如果报 `Package 'custom_msgs' not found` 或 `soem_wrapper` 缺失，先检查 `src/EcatV2_Master` 子模块和所依赖的 ROS 2 工作空间，不要把缺失消息类型理解为话题拼写问题。

> `src/soem_bringup/launch/bringup.launch.py` 的当前默认网卡为 `enp1s0`，需先用 `ip -br link` 确认 EtherCAT 网卡名称，并按机器配置检查 `rt_cpu` 和 `non_rt_cpus`。需要调整时修改该 launch 的参数后重新构建/重新 source；不要拿正在连接互联网的网卡作为 EtherCAT 主站接口。

## 2. 第一个终端：启动 EtherCAT 主站

```bash
cd ~/ZLT
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch soem_bringup bringup.launch.py
```

等待主站识别 `sn2883650`，确认 EtherCAT 从站进入预期状态、启动配置与固件 PDO 布局相符；否则先停止，不启动电机控制节点。

## 3. 第二个终端：查看四个话题

```bash
cd ~/ZLT
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic list | grep '/ecat/sn2883650/'
ros2 topic info /ecat/sn2883650/app1/read
ros2 topic info /ecat/sn2883650/app2/read
ros2 topic info /ecat/sn2883650/app3/read
ros2 topic info /ecat/sn2883650/app4/write
```

分别查看遥控器和两个 IMU：

```bash
ros2 topic echo /ecat/sn2883650/app1/read
# 另开终端查看 IMU：
ros2 topic echo /ecat/sn2883650/app2/read
ros2 topic echo /ecat/sn2883650/app3/read
```

如果某个话题没有数据，先确认主站成功创建任务、两个 CAN 端口的 IMU 已接入且报文 ID 为 1/2/3，再用 `ros2 topic type <topic>` 和 `ros2 topic hz <topic>` 检查类型及频率。

## 4. 第三个终端：启动 RC → DShot，先 dry-run

```bash
cd ~/ZLT
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch sn2031674_manual_rc manual_rc.launch.py dry_run:=true
```

诊断话题（**这里仍使用旧包/节点名，不是从站号**）：

```bash
ros2 topic echo /sn2031674_manual_rc/manual_axes
ros2 topic echo /sn2031674_manual_rc/armed
```

`manual_axes.data` 顺序为 `[throttle, yaw, roll, pitch, left_switch, right_switch, online]`。摇杆语义：`left_y` 油门、`left_x` 偏航、`right_x` 横滚、`-right_y` 俯仰。

运行日志应显示 `RC: /ecat/sn2883650/app1/read -> DShot: /ecat/sn2883650/app4/write` 且 `dry_run=True`。先检查遥控器在线标志、左右摇杆、右拨杆的位置编码与程序一致。

## 5. 拆桨后的 DShot 试验（会真实发布电机命令）

**仅在拆除全部螺旋桨、确认接线与四通道映射、油门最低、右拨杆处于锁定位置 `1`、可以独立断电、并且没有其他 DShot 发布节点时执行：**

先退出上面的 dry-run 节点（Ctrl+C），然后执行：

```bash
ros2 launch sn2031674_manual_rc manual_rc.launch.py dry_run:=false
```

检查输出：

```bash
ros2 topic echo /ecat/sn2883650/app4/write
```

确认锁定时四路均为 0；如果要进行台架验证，再按安全流程从右拨杆 `1` 切到 `3` 解锁（必须满足最低油门条件）。**程序在 `dry_run:=false` 下可能发出让电机旋转的命令。**

默认 `mode:=throttle_only`；`mode:=x_open_loop` 仅提供无 IMU 反馈的开环混控，不应用于飞行。

## 6. 常见问题

- **看到旧 `sn2031674` 话题**：检查实际启动了哪个 ROS 包，确认已 `git pull`、`colcon build`、`source install/setup.bash`，并检查 YAML 与运行日志。包名 `sn2031674_manual_rc` 留作兼容，不代表话题仍用旧序列号。
- **看不到 `sn2883650`**：检查实物从站序列号、EtherCAT 网卡、固件的 ESI/PDO 布局与主站配置；改话题名并不能改变实物从站编号。
- **IMU 不进入手动节点**：这是预期的，本包不订阅 IMU。如果需要真正基于 IMU 的闭环飞控，需单独适配两 IMU 控制器并完成姿态、符号、通道映射和安全验证。
- **6-IMU 控制器不能直接复用**：此分支中另一包的默认 6-IMU 配置与当前 4-task YAML 不一致，不能与此主站配置直接组合使用。

