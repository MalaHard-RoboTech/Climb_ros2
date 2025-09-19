# tasks/base.py
from dataclasses import dataclass
from collections import deque
from typing import Callable, Any, Optional

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import CancelResponse, GoalResponse

from cl_task_manager.action import Task

from cl_arganello_interface.msg import Imus, RopeCommand, RopeTelemetry, DebugMessage
from std_srvs.srv import Trigger
from std_msgs.msg import Float32, String

SERVICE_TIMEOUT = 5.0

def wait_future_nonblocking(node, future, timeout=SERVICE_TIMEOUT, cancel_check=None):
    start = time.time()
    while not future.done():
        if cancel_check and cancel_check():
            return None  # caller decide cosa fare
        if time.time() - start > timeout:
            return None
        time.sleep(0.01)  
    return future.result()

class HomingTask(Node):

    def __init__(self, node_name: str,
                 action_name: str = 'task_manager_server_action'):
        super().__init__(node_name)
        
        # ======= SUBSCRIBER =======
        self.winch_left_telemetry_sx = self.create_subscription(RopeTelemetry, '/winch/left/telemetry', self.telemetry_callback, 10)
        self.latest_telemetry = None
        
        # ======= PUBLISHER ========
        self.winch_left_set_motor_mode = self.create_publisher(
            String, '/winch/left/set_motor_mode', 10)
        self.winch_left_command = self.create_publisher(
            RopeCommand, '/winch/left/command', 10)
        
        # ======= CLIENT ===========
        self.client_left_brake_disengage = self.create_client(Trigger, '/winch/left/brake_disengage')
        self.client_left_brake_engage = self.create_client(Trigger, '/winch/left/brake_engage')
        self.client_left_rope_zero = self.create_client(Trigger, '/winch/left/rope_zero')
        
        # ======= ACTION SERVER =======
        self._action_server: Optional[ActionServer] = None
        self._action_name = action_name

    def telemetry_callback(self, msg):
        self.latest_telemetry = msg
        self.get_logger().info(f'Received telemetry: {msg}')

    def send_brake_command_callback(self, engage_brake):
        client = self.client_left_brake_engage if engage_brake else self.client_left_brake_disengage
        service_name = 'brake_engage' if engage_brake else 'brake_disengage'
        
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'{service_name} service not available')
            return False

        request = Trigger.Request()

        future = client.call_async(request)

        response = wait_future_nonblocking(
            self, future, timeout=5.0,
            cancel_check=lambda: False  
        )
        
        if response is None:
            self.get_logger().error(f'Failed to call {service_name} service')
            return False

        if response.success:
            self.get_logger().info(f'Brake command successful: {response.message}')
            return True

        self.get_logger().error(f'Brake command failed: {response.message}')
        return False
    
    def set_motor_mode(self, motor_mode=None):
        if motor_mode is None:
            self.get_logger().error('Motor mode not specified')
            return False
        
        msg = String()
        msg.data = motor_mode
        self.winch_left_set_motor_mode.publish(msg)
        self.get_logger().info(f'Published motor mode: {motor_mode}')
        return True   
     
    def send_rope_command(self, rope_force=0.0, rope_velocity=0.0, rope_position=0.0):
        msg = RopeCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.rope_force = rope_force
        msg.rope_velocity = rope_velocity
        msg.rope_position = rope_position
        
        self.winch_left_command.publish(msg)
        self.get_logger().info(f'Published rope command - Force: {rope_force}, Velocity: {rope_velocity}, Position: {rope_position}')
        return True

    def call_rope_zero(self):
        if not self.client_left_rope_zero.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('rope_zero service not available')
            return False

        request = Trigger.Request()
        future = self.client_left_rope_zero.call_async(request)

        response = wait_future_nonblocking(
            self, future, timeout=5.0,
            cancel_check=lambda: False
        )
        
        if response is None:
            self.get_logger().error('Failed to call rope_zero service')
            return False

        if response.success:
            self.get_logger().info(f'Rope zero successful: {response.message}')
            return True

        self.get_logger().error(f'Rope zero failed: {response.message}')
        return False

    def wait_for_peak_current(self, threshold=5.0, timeout=5.0):
        """Wait for current to reach peak indicating rope tension"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if self.latest_telemetry and abs(self.latest_telemetry.current) > threshold:
                self.get_logger().info(f'Peak current detected: {self.latest_telemetry.current}A')
                return True
            time.sleep(0.1)
        
        self.get_logger().warning(f'Peak current not reached within {timeout}s')
        return False

    # ======= TASKS =======
    def homing_task(self, goal_handle, feedback_msg):
        """
        Initialization sequence:
        1. Disengage brake
        2. Closed loop motor mode (closed_loop_torque)
        3. /winch/left/command  RopeCommand {rope_force: 20.0, rope_velocity: 0.0, rope_position: 0.0}
        4. Wait + peak current
        5. engage brake again
        6. /winch/left/command  RopeCommand {rope_force: 0.0, rope_velocity: 0.0, rope_position: 0.0}
        7. Idle
        8. send success --> trigger rope_zero
        """
        
        result = Task.Result()
        
        try:
            # Step 1: Disengage brake
            self.get_logger().info('Step 1: Disengaging brake')
            feedback_msg.status = 'Disengaging brake'
            feedback_msg.percentage = 10
            goal_handle.publish_feedback(feedback_msg)
            
            # if not self.send_brake_command_callback(False):
            #     result.success = False
            #     result.message = 'Failed to disengage brake'
            #     return result
            
            # time.sleep(20.0)  # Wait for brake to disengage
            
            # Step 2: Set closed loop motor mode
            self.get_logger().info('Step 2: Setting motor mode to closed_loop_torque')
            feedback_msg.status = 'Setting motor mode'
            feedback_msg.percentage = 25
            goal_handle.publish_feedback(feedback_msg)
            
            if not self.set_motor_mode('closed_loop_torque'):
                result.success = False
                result.message = 'Failed to set motor mode'
                return result
            
            time.sleep(1.0)  # Wait for mode change
            
            # Step 3: Send rope command with force
            self.get_logger().info('Step 3: Sending rope command with 20N force')
            feedback_msg.status = 'Applying rope tension'
            feedback_msg.percentage = 40
            goal_handle.publish_feedback(feedback_msg)
            
            if not self.send_rope_command(rope_force=30.0, rope_velocity=0.0, rope_position=0.0):
                result.success = False
                result.message = 'Failed to send rope command'
                return result
            
            # Step 4: Wait for peak current
            self.get_logger().info('Step 4: Waiting for peak current')
            feedback_msg.status = 'Waiting for rope tension'
            feedback_msg.percentage = 55
            goal_handle.publish_feedback(feedback_msg)
            
            if not self.wait_for_peak_current(threshold=1.5, timeout=2.0):
                result.success = False
                result.message = 'Peak current not detected'
                return result
            
            # Step 5: Engage brake again
            self.get_logger().info('Step 5: Engaging brake')
            feedback_msg.status = 'Engaging brake'
            feedback_msg.percentage = 70
            goal_handle.publish_feedback(feedback_msg)
            
            if not self.send_brake_command_callback(True):
                result.success = False
                result.message = 'Failed to engage brake'
                return result
            
            time.sleep(20.0)  # Wait for brake to engage
            
            #Step 6: Send zero command
            self.get_logger().info('Step 6: Sending zero rope command')
            feedback_msg.status = 'Zeroing commands'
            feedback_msg.percentage = 85
            goal_handle.publish_feedback(feedback_msg)
            
            if not self.send_rope_command(rope_force=0.0, rope_velocity=0.0, rope_position=0.0):
                result.success = False
                result.message = 'Failed to send zero command'
                return result
            
            # Step 7: Set to idle mode
            self.get_logger().info('Step 7: Setting motor to idle')
            feedback_msg.status = 'Setting idle mode'
            feedback_msg.percentage = 95
            goal_handle.publish_feedback(feedback_msg)
            
            if not self.set_motor_mode('idle'):
                result.success = False
                result.message = 'Failed to set idle mode'
                return result
            
            # Step 8: Call rope zero service
            self.get_logger().info('Step 8: Calling rope zero service')
            feedback_msg.status = 'Zeroing rope position'
            feedback_msg.percentage = 100
            goal_handle.publish_feedback(feedback_msg)
            
            if not self.call_rope_zero():
                result.success = False
                result.message = 'Failed to zero rope position'
                return result
            
            # Success
            self.get_logger().info('Homing task completed successfully')
            result.success = True
            result.message = 'Homing sequence completed successfully'
            
        except Exception as e:
            self.get_logger().error(f'Homing task failed with exception: {str(e)}')
            result.success = False
            result.message = f'Homing task failed: {str(e)}'
        
        return result