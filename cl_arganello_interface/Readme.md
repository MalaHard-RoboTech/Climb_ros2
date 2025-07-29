# 🚀 Arganello ROS2 Interface - Complete Command Reference

This document provides an updated list of **all available features and commands** for interacting with Arganello motor controllers via ROS 2.

---

## 📃 Launching Nodes

### Start node for **sx**:

```bash
ros2 run cl_arganello_interface arganello_node.py --ros-args -p serial_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5970047399-if00 -p arganello_id:=sx -p pool_rate:=200.0
```

### Start node for **dx**:

```bash
ros2 run cl_arganello_interface arganello_node.py --ros-args -p serial_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5970046081-if00 -p arganello_id:=dx -p pool_rate:=200.0
```

---

## 🚡 Control Topics

### Torque Commands:

```bash
ros2 topic pub /arganello/sx/target_torque std_msgs/msg/Float32 "{data: 0.5}" --rate 50
ros2 topic pub /arganello/dx/target_torque std_msgs/msg/Float32 "{data: 0.5}" --rate 50
```

### Velocity Commands:

```bash
ros2 topic pub /arganello/sx/target_velocity std_msgs/msg/Float32 "{data: 1.0}" --rate 50
ros2 topic pub /arganello/dx/target_velocity std_msgs/msg/Float32 "{data: 1.0}" --rate 50
```

### Position Commands:

```bash
ros2 topic pub /arganello/sx/target_position std_msgs/msg/Float32 "{data: 0.2}" --rate 50
ros2 topic pub /arganello/dx/target_position std_msgs/msg/Float32 "{data: 0.2}" --rate 50
```

---

## 🛠️ Services

### Brake Control:

```bash
ros2 service call /arganello/sx/set_brake std_srvs/srv/SetBool "{data: true}"  # Engage
ros2 service call /arganello/sx/set_brake std_srvs/srv/SetBool "{data: false}" # Release

ros2 service call /arganello/dx/set_brake std_srvs/srv/SetBool "{data: true}"  # Engage
ros2 service call /arganello/dx/set_brake std_srvs/srv/SetBool "{data: false}" # Release
```

### Motor Mode:

```bash
ros2 service call /arganello/sx/set_idle std_srvs/srv/Trigger
ros2 service call /arganello/sx/set_closed_loop std_srvs/srv/Trigger

ros2 service call /arganello/dx/set_idle std_srvs/srv/Trigger
ros2 service call /arganello/dx/set_closed_loop std_srvs/srv/Trigger
```

### Control Mode Configuration:

```bash
ros2 service call /arganello/sx/set_torque_mode std_srvs/srv/Trigger
ros2 service call /arganello/sx/set_velocity_mode std_srvs/srv/Trigger
ros2 service call /arganello/sx/set_position_mode std_srvs/srv/Trigger

ros2 service call /arganello/dx/set_torque_mode std_srvs/srv/Trigger
ros2 service call /arganello/dx/set_velocity_mode std_srvs/srv/Trigger
ros2 service call /arganello/dx/set_position_mode std_srvs/srv/Trigger
```

---

## 📶 Telemetry Topics

- `/arganello/<id>/telemetry/raw` — structured ROS message (ArganelloRawTelemetry)
- `/arganello/<id>/telemetry/raw/csv` — raw CSV string with UNIX timestamp prepended

---

## 🔹 Notes

- Replace `<id>` with `sx` or `dx` as needed.
- Only one control command (torque, velocity, position) should be active at a time.
- Telemetry is published at up to **200 Hz** from the ESP32.
- All services are synchronous and return a success message.
- Publish motor commands at **50 Hz** to not choke telemetry data
