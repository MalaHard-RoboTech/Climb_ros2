## in here you will find the ros2 nodes to expose the low level functionalities exposed via ros.

#arganello_node.py
this node connects to the arganelli via usb, it can comunicate up to 200hz and is capable of:

ros2 run cl_arganello_interface arganello_node.py --ros-args -p serial_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5970047399-if00 -p arganello_id:=sx -p loop_rate:=200.0

ros2 run cl_arganello_interface arganello_node.py --ros-args -p serial_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5970046081-if00 -p arganello_id:=dx -p loop_rate:=200.0 

🚀 Arganello ROS2 Interface

This node handles bidirectional serial communication between a PC and an ESP32-based Arganello motor controller using ROS2. It exposes topics and services for controlling motor behavior and reading telemetry.

📡 Topics
🖥️ Commands (PC → ESP32)
Topic	Type	Description
/arganello/sx/target_torque	    std_msgs/Float32	Send torque command (Nm), resets velocity & pos.
/arganello/sx/target_velocity	std_msgs/Float32	Send velocity command, resets torque & pos.
/arganello/sx/target_position	std_msgs/Float32	Send position command, resets torque & velocity.

Examples:

ros2 topic pub /arganello/sx/target_velocity std_msgs/msg/Float32 "{data: 2.0}"  # 2.0 rad/s
ros2 topic pub /arganello/sx/target_torque   std_msgs/msg/Float32 "{data: 1.5}"    # 1.5 Nm
ros2 topic pub /arganello/sx/target_position std_msgs/msg/Float32 "{data: 0.3}" --rate 20 



📶 Telemetry (ESP32 → PC)
Topic	Type	Description
/arganello/sx/brake_status	std_msgs/Bool	True if brake engaged, else False
/arganello/sx/motor_mode_status	std_msgs/Bool	True = CLOSED_LOOP, False = IDLE
/arganello/sx/encoder_count	std_msgs/Int32	Encoder tick count
/arganello/sx/iq_current	std_msgs/Float32	Phase current (Iq) in Amps
/arganello/sx/vbus_voltage	std_msgs/Float32	Input bus voltage (V)
/arganello/sx/motor_temperature	std_msgs/Float32	Motor temperature (°C)


🛠️ Services
Service Name	Type	Description
/arganello/sx/set_brake	std_srvs/SetBool	Engage (true) or release (false) the brake.
/arganello/sx/set_motor_mode	std_srvs/SetBool	Set motor to CLOSED_LOOP (true) or IDLE (false).


Examples:

ros2 service call /arganello/sx/set_brake std_srvs/srv/SetBool "{data: true}"        # Engage brake
ros2 service call /arganello/sx/set_motor_mode std_srvs/srv/SetBool "{data: true}"   # Set CLOSED_LOOP

