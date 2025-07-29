from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cl_arganello_interface',
            executable='arganello_node.py',
            name='arganello_sx',
            output='screen',
            parameters=[
                {'serial_port': '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5970047399-if00'},
                {'arganello_id': 'sx'},
                {'pool_rate': 200.0}
            ],
        ),
        Node(
            package='cl_arganello_interface',
            executable='arganello_node.py',
            name='arganello_dx',
            output='screen',
            parameters=[
                {'serial_port': '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5970046081-if00'},
                {'arganello_id': 'dx'},
                {'pool_rate': 200.0}
            ],
        ),
    ])

