
# CAN Manager

> ROS2-based CAN communication manager for autonomous driving systems.  
> Handles message parsing, relaying, and control command publishing between ROS and CAN interfaces.

---

## 🧩 Components

### 🔄 C2R (CAN to ROS)
- Receives CAN data
- Publishes ROS2 messages (e.g., MCUStatus2, BMSPackStatus)
- Converts raw values using factor/offset

### 🔁 R2C (ROS to CAN)
- Listens to ROS control commands
- Sends CAN commands (accelerate, steer, brake, body signals)

---

## 📦 Architecture

```
rclcpp::Node (CanManager)
├── 📤 R2C → ROS ➝ CAN
│   ├── AdControlAccelerate
│   ├── AdControlSteering
│   └── AdControlBrake / Body
│
└── 📥 C2R → CAN ➝ ROS
    ├── MCUStatus2
    ├── BMSPackStatus
    └── MCUGeneralStatus
```

---

## ⚙️ Parameters

Parameters are fully configurable through ROS2's parameter server.  
See `declare_parameters()` in `node.cpp` for full list.

---

## 🚀 How to Use

```bash
colcon build
source install/setup.bash
ros2 run can_manager can_manager_node
```

---

## 🧪 Doxygen

This documentation is auto-generated with Doxygen.

```bash
doxygen Doxyfile
xdg-open docs/html/index.html
```

---

## 📧 Authors

- **reidlo** (naru5135@wavem.net)
- **changunAn** (changun516@wavem.net)
