#!/usr/bin/env python3
  
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor

from cl_task_manager.action import Task
from cl_arganello_interface.msg import ArganelloEnhancedTelemetry
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import Float32  

from tasks import task_0  

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager_server')
        
        # Initialize variable
        
        # SUBCRIBER -----------------
        self.telemetry_subscriber = self.create_subscription(
            ArganelloEnhancedTelemetry,
            '/arganello/dx/telemetry/enhanced',
            self.telemetry_callback,
            10
        )
        # PUBLISHERS -----------------
        self.velocity_publisher = self.create_publisher(Float32, '/arganello/dx/target_velocity',10)
        self.torque_publisher = self.create_publisher(Float32, '/arganello/dx/target_torque', 10)
        
        # CLIENT ---------------------
        self.client_set_brake = self.create_client(SetBool, 'arganello/dx/set_brake')
        self.client_set_closed_loop = self.create_client(Trigger,'/arganello/dx/set_closed_loop')
        self.client_set_velocity_mode = self.create_client(Trigger,'/arganello/dx/set_velocity_mode')
        self.client_set_idle = self.create_client(Trigger,'/arganello/dx/set_idle')

        # Publishers for velocity and torque
        self.action_server = ActionServer(self, 
                                          Task, 
                                          'task_manager_server_action', 
                                          self.execute_callback,
                                          cancel_callback=self.cancel_callback)
        self.latest_telemetry = None

        # Initialize a list to store telemetry data
        self.telemetry_data = []

        self.get_logger().info('Task Server Node has been started.')

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    # EXECUTION TASKS -----------------
    def execute_callback(self, goal_handle):
        self.get_logger().info('recive goal request: %s' % goal_handle.request.task_number)
        
        feedback_msg = Task.Feedback()

        # TASK 0
        if goal_handle.request.task_number == 0:
            self.get_logger().info('Task 0 started')
            #example
            task_0(feedback_msg, goal_handle)    

        # TASK 1
        elif goal_handle.request.task_number == 1:
            self.get_logger().info('Task 1 started - Brake Disengage')
            
            feedback_msg.percentage = 10
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info("Disengaging brake...")
            
            if self.send_brake_command_callback(False):
                self.get_logger().info('Brake command sent successfully')
                feedback_msg.percentage = 50
                goal_handle.publish_feedback(feedback_msg)
                
                self.get_logger().info('Brake disengaged successfully - confirmed by telemetry')
                feedback_msg.percentage = 100
                goal_handle.publish_feedback(feedback_msg)
            
            else:
                self.get_logger().error('Failed to send brake command')
                goal_handle.abort()
                result = Task.Result()
                result.success = False
                return result
         
        # TASK 2
        elif goal_handle.request.task_number == 2:
            self.get_logger().info('Task 2 started - Brake Engage')
            
            feedback_msg.percentage = 10
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info("Engaging brake...")
            
            if self.send_brake_command_callback(True):
                self.get_logger().info('Brake command sent successfully')
                feedback_msg.percentage = 50
                goal_handle.publish_feedback(feedback_msg)
                
                # Wait for brake status to become true
                self.get_logger().info('Waiting for brake status confirmation...')
                if self.wait_for_brake_status(True, timeout=10.0):
                    self.get_logger().info('Brake engaged successfully - confirmed by telemetry')
                    feedback_msg.percentage = 100
                    goal_handle.publish_feedback(feedback_msg)
                else:
                    self.get_logger().error('Timeout waiting for brake status confirmation')
                    goal_handle.abort()
                    result = Task.Result()
                    result.success = False
                    return result
            else:
                self.get_logger().error('Failed to send brake command')
                goal_handle.abort()
                result = Task.Result()
                result.success = False
                return result
        
        # TASK 3 
        elif goal_handle.request.task_number == 3:
            self.get_logger().info('Task 3 started - INITIALIZE TASK')
            
            feedback_msg.percentage = 10
            goal_handle.publish_feedback(feedback_msg)
            
            if self.initialization_step():
                feedback_msg.percentage = 100
                goal_handle.publish_feedback(feedback_msg)
            else:
                self.get_logger().error('Initialization failed')
                goal_handle.abort()
                result = Task.Result()
                result.success = False
                return result
            
        # INVALID TASK NUMBER
        else:
            self.get_logger().error("Invalid Task Number")
            goal_handle.abort()
            result = Task.Result()
            result.success = False
            return result
        
        self.get_logger().info("Goal succeeded")
        goal_handle.succeed()
        result = Task.Result()
        result.success = True
        return result 
    
    # CALLBACK --------------------------------
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
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Brake command successful: {response.message}')
                return True
            else:
                self.get_logger().error(f'Brake command failed: {response.message}')
                return False
        else:
            self.get_logger().error('Failed to call brake service')
            return False
        
    def trigger_service_callback(self, client,service_name):
        
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Set Closed Loop service not available')
            return False
        
        request = Trigger.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info('Set ', service_name ,' successful')
                return True
            else:
                self.get_logger().error(f'Set ', service_name ,' failed: {response.message}')
                return False
        else:
            self.get_logger().error('Failed to call Set ', service_name ,' service')
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

    # TASK COMPUTATION --------------------------------
    def wait_for_sync_roller_stabilization(self, timeout=4.0, stable_duration=2.0, tolerance=5):
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
    
    def initialization_step(self):
        """
        Initialization sequence:
        1. Set brake to false
        2. Send closed loop trigger
        3. Set target velocity to 0.5
        4. Wait 2 seconds
        5. Set brake to false (again)
        6. Set velocity to 0
        7. Send idle trigger
        """
        self.get_logger().info('Starting initialization sequence...')
        
        # Step 1: Set brake to false
        self.get_logger().info('Step 1: Disengaging brake...')
        if not self.send_brake_command_callback(False):
            self.get_logger().error('Failed to disengage brake in initialization')
            return False
        
        # Step 2: Send closed loop trigger
        self.get_logger().info('Step 2: Setting closed loop mode...')
        
        if not self.trigger_service_callback(self.client_set_closed_loop,'closed_loop'):
            self.get_logger().error('Failed to set closed loop mode')
            return False
        
        # Step 2.5: Send velocity mode trigger
        self.get_logger().info('Step 2.5: Setting velocity mode...')
        if not self.trigger_service_callback(self.client_set_velocity_mode,'velocity_mode'):
            self.get_logger().error('Failed to set velocity mode')
            return False
        
        # Step 3: Set target velocity to 0.5
        self.get_logger().info('Step 3: Setting target velocity to 0.5...')
        self.publisher_command('velocity_mode',0.5)
        
        
        # # Step 4: Wait 2 seconds
        # self.get_logger().info('Step 4: Waiting 2 seconds...')
        # time.sleep(2.0)
        # Step 4: Wait for sync_roller_raw to stabilize
        self.get_logger().info('Step 4: Waiting for sync_roller_raw to stabilize...')
        if not self.wait_for_sync_roller_stabilization():
            self.get_logger().error('Failed to achieve sync_roller_raw stabilization')
            return False
        
        # Step 5: Set brake to false (again)
        self.get_logger().info('Step 5: engaging brake again...')
        if not self.send_brake_command_callback(True):
            self.get_logger().error('Failed to engaging brake in step 5')
            return False
        
        # Step 6: Set velocity to 0
        self.get_logger().info('Step 6: Setting velocity to 0...')
        self.publisher_command('velocity_mode',0.0)
        
        # Step 7: Send idle trigger
        self.get_logger().info('Step 7: Setting idle mode...')
        if not self.trigger_service_callback(self.client_set_idle,'idle_mode'):
            self.get_logger().error('Failed to set idle mode')
            return False
        
        self.get_logger().info('Initialization sequence completed successfully')
        return True

    
def main(args=None):
    rclpy.init(args=args)
    
    try:
        minimal_action_server = TaskManager()

        # Use a MultiThreadedExecutor to enable processing goals concurrently
        executor = MultiThreadedExecutor()

        rclpy.spin(minimal_action_server, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        
#to tested: 
#ros2 action send_goal /task_manager_server_action cl_task_manager/action/Task "{task_number: 3}" -f
