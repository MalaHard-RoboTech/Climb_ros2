from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cl_arganello_interface',
            executable='encoder_node',   
            name='encoder_node',
            output='screen',
            parameters=[],
        ) 
    ])
    