# ur10e_keyboard_control_ursim_ros2

![Демонстрация](images/output.gif)

# UR10e Keyboard Control via ROS 2 and URSim on VirtualBox

This package enables real-time keyboard control of the UR10e robotic manipulator running in **URSim (simulator)** on **VirtualBox**, using **ROS 2 Humble** running on the host machine.

## 🛠️ System Overview

- Host machine runs ROS 2 and sends trajectory commands.
- Virtual machine (VirtualBox) runs URSim with External Control URCap.
- Network communication is done over **Bridged Adapter**.
- The robot receives joint trajectory commands via `joint_trajectory_controller`.

[Host PC - ROS 2] <--network--> [VirtualBox - URSim with External Control]

---

## ✅ Requirements

- Ubuntu 22.04 on host machine
- ROS 2 Humble installed
- URSim running inside VirtualBox
- External Control URCap installed in URSim
- Bridged network adapter enabled in VirtualBox
- `ur_robot_driver` installed and configured
- Controller: `joint_trajectory_controller`

---

## 📦 Package Structure
