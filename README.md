# Climb_ros2
here you can find all about ros2 for climb robot alpine
we have implemert 2 arduino sketches donlge_espnow_ros2_bridge that sends and recives serial command and sends them to the arganelli via espnow, for now just the dx one
esp32_espnow_arganello_dx_e_sx is the firmawer intended to go on the esp32 onboard the arganelli, they have to be the same to eliminate code duplication



##arganello interface
ros2 run cl_arganello_interface encoder_node.py 
this node is responsable to subscribe to the commands to send via serial, and it will foward it to the dongle, also recives the encoder counts and publishes the transformation for rviz2

ros2 topic pub /arganello_dx_command std_msgs/String "{data: 'off'}"
ros2 topic pub /arganello_dx_command std_msgs/String "{data: 'on'}"
🔄 Encoder count: 563 that will be tranformed in meter and published by 


