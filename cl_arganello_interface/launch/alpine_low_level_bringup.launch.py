from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package share directory for relative paths
    pkg_share = get_package_share_directory('cl_arganello_interface')
    config_path = os.path.join(pkg_share, 'config', 'arganelloTelemetry.json')
    
    return LaunchDescription([
        # Left telemetry node
        Node(
            package='cl_arganello_interface',
            executable='telemetry_node.py',
            name='telemetry_node_left',
            parameters=[{
                'side': 'left',
                'serial_port': '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5970047399-if00',
                'config_path': config_path,
                'debug_mode': True
            }]
        ),
        
        # Right telemetry node
        Node(
            package='cl_arganello_interface',
            executable='telemetry_node.py',
            name='telemetry_node_right',
            parameters=[{
                'side': 'right',
                'serial_port': '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5970046081-if00',
                'config_path': config_path,
                'debug_mode': True
            }]
        ),
        
        # Dongle node
        Node(
            package='cl_arganello_interface',
            executable='dongle_node.py',
            name='dongle_node',
            parameters=[{
                'serial_port': '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A010904-if00',
                'baud': 1000000,
                'poll_rate': 100.0
            }]
        ),
        
        # Jump node
        Node(
            package='cl_arganello_interface',
            executable='jump.py',
            name='jump_node'
        ),
    ])