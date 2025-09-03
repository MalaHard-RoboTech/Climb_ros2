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

from cl_arganello_interface.msg import ArganelloEnhancedTelemetry
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import Float32

SERVICE_TIMEOUT = 5.0

def wait_future_nonblocking(node, future, timeout=SERVICE_TIMEOUT, cancel_check=None):
    start = time.time()
    while not future.done():
        if cancel_check and cancel_check():
            return None  # caller decide cosa fare
        if time.time() - start > timeout:
            return None
        time.sleep(0.01)  # piccolo yield; l'executor continua a processare altrove
    return future.result()

class BaseTasks(Node):

    def __init__(self, node_name: str,
                 action_name: str = 'task_manager_server_action'):
        super().__init__(node_name)
        
        # ======= SUBSCRIBER =======
        self.latest_telemetry: Optional[ArganelloEnhancedTelemetry] = None      
        self.telemetry_data = deque(maxlen=10000)
        self.telemetry_subscriber = self.create_subscription(
            ArganelloEnhancedTelemetry,
            '/arganello/dx/telemetry/enhanced',
            self.telemetry_callback,
            10
        )

        # ======= PUBLISHER ========
        self.velocity_publisher = self.create_publisher(
            Float32, '/arganello/dx/target_velocity', 10
        )
        self.torque_publisher = self.create_publisher(
            Float32, '/arganello/dx/target_torque', 10
        )

        # ======= CLIENT ===========
        self.client_set_brake = self.create_client(SetBool, 'arganello/dx/set_brake')
        self.client_set_closed_loop = self.create_client(Trigger, '/arganello/dx/set_closed_loop')
        self.client_set_velocity_mode = self.create_client(Trigger, '/arganello/dx/set_velocity_mode')
        self.client_set_idle = self.create_client(Trigger, '/arganello/dx/set_idle')

        # ======= ACTION SERVER =======
        self._action_server: Optional[ActionServer] = None
        self._action_name = action_name

    def telemetry_callback(self, msg):
        self.latest_telemetry = msg
        self.telemetry_data.append(msg)
        # self.get_logger().info(f'Received telemetry: {msg}')

    def send_brake_command_callback(self, engage_brake):
        
        if not self.client_set_brake.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Brake service not available')
            return False

        request = SetBool.Request()
        request.data = engage_brake

        future = self.client_set_brake.call_async(request)
        # rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        response = wait_future_nonblocking(
            self, future, timeout=5.0,
            cancel_check=lambda: False  # volendo: goal_handle.is_cancel_requested
        )
        
        if response is None:
            self.get_logger().error('Failed to call brake service')
            return False

        # response = future.result()
        if response.success:
            self.get_logger().info(f'Brake command successful: {response.message}')
            return True

        self.get_logger().error(f'Brake command failed: {response.message}')
        return False

    def trigger_service_callback(self, client, service_name):
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'{service_name} service not available')
            return False

        request = Trigger.Request()
        future = client.call_async(request)
        # rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        response = wait_future_nonblocking(
            self, future, timeout=5.0,
            cancel_check=lambda: False  # volendo: goal_handle.is_cancel_requested
        )
        
        if future.result() is None:
            self.get_logger().error(f'Failed to call {service_name} service')
            return False

        response = future.result()
        if response.success:
            self.get_logger().info(f'Set {service_name} successful')
            return True

        self.get_logger().error(f'Set {service_name} failed: {response.message}')
        return False
            
    def publisher_command(self, command_name, value=0.0):
        msg = Float32()
        msg.data = value
        if command_name == 'velocity_mode':
            self.velocity_publisher.publish(msg)
        elif command_name == 'torque_mode':
            self.torque_publisher.publish(msg)
        # elif command_name == 'position_mode':
        #     self.position_publisher.publish(msg)
        self.get_logger().info(f'Published velocity: {value}')
        
    def wait_for_sync_roller_stabilization(self, timeout=30.0, stable_duration=2.0, tolerance=5):
        start_time = time.time()
        stable_start_time = None
        last_value = None
        
        while time.time() - start_time < timeout:
            if self.latest_telemetry is None:
                time.sleep(0.1)
                continue
                
            current_value = self.latest_telemetry.sync_roller_raw
            current_time = time.time()
            
            if last_value is None:
                last_value = current_value
                stable_start_time = current_time
            else:
                # Check if value is within tolerance range
                if abs(current_value - last_value) <= tolerance:
                    # Value is stable, check if we've been stable long enough
                    if stable_start_time is None:
                        stable_start_time = current_time
                    elif current_time - stable_start_time >= stable_duration:
                        self.get_logger().info(f'sync_roller_raw stabilized at {current_value} for {stable_duration} seconds')
                        return True
                else:
                    # Value changed beyond tolerance, reset stable timer
                    stable_start_time = None
                    last_value = current_value
                    self.get_logger().info(f'sync_roller_raw changed to {current_value}, resetting stability timer')
            time.sleep(0.1)
        self.get_logger().error(f'Timeout waiting for sync_roller_raw stabilization after {timeout} seconds')
        return False

    # ======= TASKS =======
    def initialization_task(self, goal_handle, feedback_msg):
        """
        Initialization sequence:
        1. Disengage brake
        2. Closed loop
        2.5 Velocity mode
        3. Velocity 0.5
        4. Wait + stabilize sync_roller_raw
        5. Disengage brake again
        6. Velocity 0
        7. Idle
        """
        step_total = 7
        def publish_step(step_idx, msg):
            perc = int((step_idx / step_total) * 100)
            feedback_msg.percentage = perc
            self.get_logger().info(f'[initialization_task] {msg} ({perc}%)')
            goal_handle.publish_feedback(feedback_msg)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = Task.Result()
            result.success = False
            self.get_logger().warn('[initialization_task] Canceled before start')
            return result

        result = Task.Result()
        publish_step(0, 'Disengaging brake')
        if not self.send_brake_command_callback(False):
            raise RuntimeError('Brake disengage failed (step 1)')

        publish_step(1, 'Setting closed loop')
        if not self.trigger_service_callback(self.client_set_closed_loop, 'closed_loop'):
            raise RuntimeError('Closed loop failed (step 2)')

        publish_step(2, 'Setting velocity mode')
        if not self.trigger_service_callback(self.client_set_velocity_mode, 'velocity_mode'):
            raise RuntimeError('Velocity mode failed (step 2.5)')

        publish_step(3, 'Setting target velocity 0.5')
        self.publisher_command('velocity_mode', 0.5)

        publish_step(4, 'Waiting 2s + stabilization')
        import time as _t
        _t.sleep(2.0)
        #non posso provarlo
        # if not self.wait_for_sync_roller_stabilization():
        #     raise RuntimeError('Stabilization timeout (step 4)')

        publish_step(5, 'Disengaging brake again')
        if not self.send_brake_command_callback(False):
            raise RuntimeError('Brake disengage failed (step 5)')

        publish_step(6, 'Setting velocity 0 and idle')
        self.publisher_command('velocity_mode', 0.0)
        if not self.trigger_service_callback(self.client_set_idle, 'idle_mode'):
            raise RuntimeError('Idle mode failed (step 7)')
        
        # Complete (100%)
        feedback_msg.percentage = 100
        goal_handle.publish_feedback(feedback_msg)
        result.success = True
        self.get_logger().info('[initialization_task] Completed (100%)')
        
        return result