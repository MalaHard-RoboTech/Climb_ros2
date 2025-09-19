#!/usr/bin/env python3
# # Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time

from cl_task_manager.action import Task

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class MinimalActionServer(Node):

    def __init__(self):
        super().__init__('minimal_action_server')

        self._action_server = ActionServer(
            self,
            Task,
            'fibonacci',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute a goal."""
        self.get_logger().info('recive goal request: %s' % goal_handle.request.task_number)
        
        feedback_msg = Task.Feedback()

        # TASK 0
        if goal_handle.request.task_number == 0:
            self.get_logger().info('Task 0 started')
        
        # TASK 1  
        elif goal_handle.request.task_number == 1:
            self.get_logger().info('Task 1 started: Fibonacci sequence')

            # Fixed number of Fibonacci terms
            order = 10
            # feedback_msg.progress = [0, 1]  # Comment out until we know the correct field name
            fibonacci_sequence = [0, 1]

            for i in range(1, order - 1):  # Already have two elements
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
                    result = Task.Result()
                    result.success = False
                    return result

                # Compute next Fibonacci number
                next_number = fibonacci_sequence[i] + fibonacci_sequence[i - 1]
                fibonacci_sequence.append(next_number)

                # Log feedback (without publishing until we know the correct field)
                self.get_logger().info(f'Fibonacci sequence: {fibonacci_sequence}')
                # goal_handle.publish_feedback(feedback_msg)  # Comment out for now

                time.sleep(1)  # Simulate delay

            self.get_logger().info("Fibonacci Task completed")
            
        elif goal_handle.request.task_number == 2:
            self.get_logger().info('Task 2 started') 

        self.get_logger().info("Goal succeeded")
        goal_handle.succeed()
        result = Task.Result()
        result.success = True
        return result 


def main(args=None):
    rclpy.init(args=args)
    
    try:
        minimal_action_server = MinimalActionServer()

        # Use a MultiThreadedExecutor to enable processing goals concurrently
        executor = MultiThreadedExecutor()

        rclpy.spin(minimal_action_server, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()