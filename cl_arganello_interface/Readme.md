# 🚀 Arganello ROS2 Interface - Complete Command Reference

This document provides an updated list of **all available features and commands** for interacting with Arganello motor controllers via ROS 2.

---

#Telemetry_node

this node interfaces both for the left and righ winch firmware via it's flag and configurations
it send a list of neede telemetry values and send it at the beginning for high rsate telemetry without the asking overhead, it runs at 200 hz

### Start node for **sx** and  **dx** :

we specify the package, node as usal, then we have the flagt that automatically assigns the correct namespace "left"/"right", the serial id of the microcontroller (MCU), if using this will autoconnect even if changing the usb port; the config path is used to inject a list of high rate telemetry to the wiches MCU so you can request any information from the odrive without touching the mcu code. debig mode is to iject comands directly to the winch MCU, just for testing and debug. 

```bash

ros2 run cl_arganello_interface telemetry_node.py --ros-args -p side:=left -p serial_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5970047399-if00 -p config_path:=/home/msi/Desktop/ros2_ws/src/cl_arganello_interface/config/arganelloTelemetry.json -p debug_mode:=true

ros2 run cl_arganello_interface telemetry_node.py --ros-args -p side:=right -p serial_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5970046081-if00 -p config_path:=/home/msi/Desktop/ros2_ws/src/cl_arganello_interface/config/arganelloTelemetry.json -p debug_mode:=true

```

### Motor Control Mode

Select which control mode (torque / velocity / position) the closed loop will use:

```bash

ros2 topic pub --once /winch/left/set_motor_mode std_msgs/msg/String "{data: 'idle'}"
ros2 topic pub --once /winch/left/set_motor_mode std_msgs/msg/String "{data: 'closed_loop_torque'}"
ros2 topic pub --once /winch/left/set_motor_mode std_msgs/msg/String "{data: 'closed_loop_velocoty'}"
ros2 topic pub --once /winch/left/set_motor_mode std_msgs/msg/String "{data: 'closed_loop_position'}"

ros2 topic pub --once /winch/right/set_motor_mode std_msgs/msg/String "{data: 'idle'}"
ros2 topic pub --once /winch/right/set_motor_mode std_msgs/msg/String "{data: 'closed_loop_torque'}"
ros2 topic pub --once /winch/right/set_motor_mode std_msgs/msg/String "{data: 'closed_loop_velocoty'}"
ros2 topic pub --once /winch/right/set_motor_mode std_msgs/msg/String "{data: 'closed_loop_position'}"

```

### Brake Control 

To engage and sisengage the motor

```bash

ros2 service call /winch/left/brake_engage std_srvs/srv/Trigger "{}"
ros2 service call /winch/left/brake_disengage std_srvs/srv/Trigger "{}"

ros2 service call /winch/right/brake_engage std_srvs/srv/Trigger "{}"
ros2 service call /winch/right/brake_disengage std_srvs/srv/Trigger "{}"

```

### Rope control 

given the selected motor mode it will discard the other fileds
closed_loop_torque -> rope_force
closed_loop_velocoty -> rope_velocity
closed_loop_position -> rope_position

```bash

ros2 topic pub --rate 100 /winch/left/command cl_arganello_interface/msg/RopeCommand "{rope_force: 10.0, rope_velocity: 0.0, rope_position: 0.0}"
ros2 topic pub --rate 100 /winch/right/command cl_arganello_interface/msg/RopeCommand "{rope_force: 10.0, rope_velocity: 0.0, rope_position: 0.0}"

ros2 topic pub --once /winch/left/command cl_arganello_interface/msg/RopeCommand "{rope_force: 10.0, rope_velocity: 0.0, rope_position: 0.0}"
ros2 topic pub --once /winch/right/command cl_arganello_interface/msg/RopeCommand "{rope_force: 10.0, rope_velocity: 0.0, rope_position: 0.0}"

```

Output topics

```bash

ros2 topic echo /winch/left/telemetry/debug
ros2 topic echo /winch/left/telemetry/csv

```

Debug with plotjuggler:

```bash

ros2 run plotjuggler plotjuggler

```


--- 

# Friction estimator
is the ros2 node used to estimate the friction of the winches experimentally

--- 

# Dongle Node

The **dongle_node** bridges the USB serial interface of the **dongle ESP32** to ROS 2 topics.  
The dongle forwards commands to the onboard Alpine body microcontroller and relays telemetry back.  
This node wraps the low-level serial protocol into ROS 2 topics, making it easy to integrate with the rest of the system.

---

## Features

- Opens a serial connection to the dongle ESP32 (`serial_port`, `baud`).
- Converts ROS 2 topics into serial commands:
  - `/alpine/dongle/motorSpeed` → `m<val>`
  - `/alpine/dongle/servoValve1` → `s1 <deg>`
  - `/alpine/dongle/servoValve2` → `s2 <deg>`
- Reads serial data, reconstructs complete CSV lines, and republishes:
  - `/alpine/dongle/telemetry/raw` (`std_msgs/String`): unmodified CSV strings.
  - `/alpine/dongle/telemetry` (`std_msgs/Float32MultiArray`): parsed structure `[epoch_ms, imu1[11], imu2[11]]`.

---

## Parameters

| Name         | Type   | Default       | Description                              |
|--------------|--------|---------------|------------------------------------------|
| `serial_port`| string | `/dev/ttyUSB0`| Serial device path for the dongle ESP32. |
| `baud`       | int    | `1000000`     | Serial baudrate.                         |
| `poll_rate`  | float  | `200.0`       | Polling frequency in Hz.                 |

---

## Topics

### Publishers
- **`/alpine/dongle/telemetry/raw`** (`std_msgs/String`)  
  Raw CSV lines from the dongle, de-chunked.

- **`/alpine/dongle/telemetry`** (`std_msgs/Float32MultiArray`)  
  Parsed structured telemetry.  
  Format: `[epoch_ms, imu1[0..10], imu2[0..10]]`.

### Subscribers
- **`/alpine/dongle/motorSpeed`** (`std_msgs/Float32`) → `m<val>`  
- **`/alpine/dongle/servoValve1`** (`std_msgs/Float32`) → `s1 <deg>`  
- **`/alpine/dongle/servoValve2`** (`std_msgs/Float32`) → `s2 <deg>`  

---

## Example Usage

Start the node with a persistent USB device path:

```bash

ros2 run cl_arganello_interface dongle_node.py   --ros-args   -p serial_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A010904-if00   -p baud:=1000000   -p poll_rate:=200.0

```

in this configuaration it can send at recive commands at 100hz.

Publish commands:

```bash
# Set motor speed
ros2 topic pub -1 /alpine/dongle/motorSpeed std_msgs/msg/Float32 "{data: 0.3}"

# Move servo valve 1
ros2 topic pub -1 /alpine/dongle/servoValve1 std_msgs/msg/Float32 "{data: 45.0}"

# Move servo valve 2
ros2 topic pub -1 /alpine/dongle/servoValve2 std_msgs/msg/Float32 "{data: 30.0}"
```

Listen to telemetry:

```bash
ros2 topic echo /alpine/dongle/telemetry/raw
ros2 topic echo /alpine/dongle/telemetry

```






# ros2 run cl_arganello_interface jump.py
just a small demonstation of jumping for the event,  aopen loop preprpgrammed sequance of servo valve positions for a simple jump and landing

```bash

ros2 run cl_arganello_interface jump.py

```

```bash

ros2 service call /alpine/jump std_srvs/srv/Trigger

```

