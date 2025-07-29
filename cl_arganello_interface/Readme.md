# 🚀 Arganello ROS2 Interface

This package contains ROS2 nodes to interface with the **Arganello motor controllers** via USB using serial communication. Each node handles **bidirectional communication** at up to **200Hz**, exposing control topics and telemetry feedback.

---

## 🧠 Node: `arganello_node.py`

This node connects to an ESP32-based Arganello over USB and exposes ROS2 interfaces for control and monitoring.

### 🏃 Run Examples

Start the node for the **left (sx)** motor:
```bash
ros2 run cl_arganello_interface arganello_node.py --ros-args -p serial_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5970047399-if00 -p arganello_id:=sx -p pool_rate:=200.0
```

Start the node for the **right (dx)** motor:
```bash
ros2 run cl_arganello_interface arganello_node.py --ros-args -p serial_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5970046081-if00 -p arganello_id:=dx -p pool_rate:=200.0
```

---

## 📡 Topics

### 🖥️ Commands (PC → ESP32)

| Topic                              | Type               | Description                                      |
|-----------------------------------|--------------------|--------------------------------------------------|
| `/arganello/<id>/target_torque`   | `std_msgs/Float32` | Send torque command (Nm), resets vel & pos       |
| `/arganello/<id>/target_velocity` | `std_msgs/Float32` | Send velocity command (rad/s), resets torque/pos |
| `/arganello/<id>/target_position` | `std_msgs/Float32` | Send position command (rad), resets torque/vel   |

#### 🧪 Examples
```bash
ros2 topic pub /arganello/sx/target_velocity std_msgs/msg/Float32 "{data: 2.0}"   # 2.0 rad/s
ros2 topic pub /arganello/sx/target_torque   std_msgs/msg/Float32 "{data: 1.5}"   # 1.5 Nm
ros2 topic pub /arganello/sx/target_position std_msgs/msg/Float32 "{data: 0.3}" --rate 20  # 0.3 rad @ 20Hz
```

---

### 📶 Telemetry (ESP32 → PC)

| Topic                                | Type               | Description                         |
|-------------------------------------|--------------------|-------------------------------------|
| `/arganello/<id>/brake_status`      | `std_msgs/Bool`    | True if brake is engaged            |
| `/arganello/<id>/motor_mode_status` | `std_msgs/Bool`    | True = CLOSED_LOOP, False = IDLE    |
| `/arganello/<id>/encoder_count`     | `std_msgs/Int32`   | Encoder tick count                  |
| `/arganello/<id>/iq_current`        | `std_msgs/Float32` | Phase current (Iq) in Amps          |
| `/arganello/<id>/vbus_voltage`      | `std_msgs/Float32` | Input bus voltage in Volts          |
| `/arganello/<id>/motor_temperature` | `std_msgs/Float32` | Motor temperature in Celsius (°C)   |

---

## 🛠️ Services

| Service Name                         | Type                 | Description                                        |
|-------------------------------------|----------------------|----------------------------------------------------|
| `/arganello/<id>/set_brake`         | `std_srvs/SetBool`   | Engage (`true`) or release (`false`) the brake     |
| `/arganello/<id>/set_motor_mode`    | `std_srvs/SetBool`   | CLOSED_LOOP (`true`) or IDLE (`false`) motor mode  |

#### 🧪 Examples
```bash
ros2 service call /arganello/sx/set_brake std_srvs/srv/SetBool "{data: true}"         # Engage brake
ros2 service call /arganello/sx/set_motor_mode std_srvs/srv/SetBool "{data: true}"    # Set to CLOSED_LOOP mode
```

---

## 📍 Notes

- Replace `<id>` with `sx`, `dx`, or your custom identifier.
- Only one command (torque, velocity, or position) should be active at a time. Sending one resets the others.
- Make sure the serial port path is correct (`/dev/serial/by-id/...`) before launching the node.

---
