<div align="center">

# ZLT

**EtherCAT Quadrotor Control & Multi-IMU Structural Damping Stack**

<p>
  <img src="https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white" alt="ROS 2 Humble">
  <img src="https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white" alt="Ubuntu 22.04">
  <img src="https://img.shields.io/badge/EtherCAT-SOEM-2563EB" alt="SOEM EtherCAT">
  <img src="https://img.shields.io/badge/IMU-6x%20HIPNUC-6F42C1" alt="6x HIPNUC IMU">
</p>

<img src="docs/assets/zlt_hero.jpg" width="820" alt="ZLT quadrotor research platform">

</div>

## Demo

<div align="center">
  <img src="docs/assets/demo.gif" width="760" alt="ZLT flight demo">
  <br>
  <sub>Real-flight demo</sub>
</div>

## Quick Start

```bash
git clone --recurse-submodules https://github.com/ssybh2/ZLT.git
cd ZLT

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch soem_bringup bringup.launch.py
ros2 launch soft_drone_manual_controller manual_controller.launch.py dry_run:=true
```

> Before first motor-enabled testing, verify the EtherCAT interface, IMU directions, motor order and arming logic with propellers removed.

---

<div align="center">
  <sub>Soft Robotics · Aerial Robotics · ROS 2 · EtherCAT</sub>
</div>
