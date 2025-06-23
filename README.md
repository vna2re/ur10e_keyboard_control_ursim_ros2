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
ur10e_rviz_control/

├── launch/

│ └── display.launch.py

├── scripts/

│ ├── keyboard_control.cpp

├── config/

│ └── joint_trajectory_controller.yaml

├── urdf/

│ └── ur10e.urdf

├── CMakeLists.txt

└── package.xml

---

## ▶️ Running the System

### 1. Start URSim inside VirtualBox

- Ensure **URSim is in Remote Control** mode.
- In URSim, open a Program and insert **External Control URCap**.
- Set `Host IP` to your host machine’s IP (e.g., `192.168.31.254`).
- Press **Play** in the Program tab to activate the External Control.

### 2. Launch the robot driver from host (ROS 2)

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=192.168.31.216 launch_rviz:=true
```

Parameters:

  - ur_type: Set to your robot type (ur10e, ur5e, etc.)
  - robot_ip: IP address of URSim (check via ip a in URSim terminal)
  - launch_rviz: Optionally launch RViz for visualization
  🛑 Important: The IP address must match the one configured in URSim.

### 3. Activate the trajectory controller

ros2 control set_controller_state joint_trajectory_controller active

    If you’re using scaled_joint_trajectory_controller, adjust the name accordingly.

### 4. Run the keyboard control node

```
ros2 run ur10e_rviz_control keyboard_control
```

### Keyboard Bindings

    → / ←: Select next/previous joint

    ↑ / ↓: Increase/decrease joint angle

    q: Exit the program

This node publishes trajectory_msgs/msg/JointTrajectory to /joint_trajectory_controller/joint_trajectory.
