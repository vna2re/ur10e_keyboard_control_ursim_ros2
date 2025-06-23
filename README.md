# ur10e_keyboard_control_ursim_ros2

# UR10e Keyboard Control via ROS 2 and URSim on VirtualBox

This package enables real-time keyboard control of the UR10e robotic manipulator running in **URSim (simulator)** on **VirtualBox**, using **ROS 2 Humble** running on the host machine.

## 🛠️ System Overview

- Host machine runs ROS 2 and sends trajectory commands.
- Virtual machine (VirtualBox) runs URSim with External Control URCap.
- Network communication is done over **Bridged Adapter**.
- The robot receives joint trajectory commands via `joint_trajectory_controller`.

