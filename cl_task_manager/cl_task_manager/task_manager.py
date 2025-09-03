#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.action import CancelResponse, GoalResponse

from cl_task_manager.action import Task

from tasks.task_base import BaseTasks
from tasks.tasks import task_0

class TaskManager(BaseTasks):
    def __init__(self):
        super().__init__('task_manager_server')
        self.action_server = self._create_action_server()
        # Dizionario task: ogni callable deve avere signature (goal_handle, feedback_msg) -> Task.Result
        self.tasks = {
            0: lambda gh, fb: task_0(self, gh, fb),  # task di TEST
            1: self.initialization_task,    # task di Calibration
        }
        self.get_logger().info('Task Action Server Node has been started.')



    def _create_action_server(self):
        from rclpy.action import ActionServer
        return ActionServer(
            self,
            Task,
            'task_manager_server_action',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def destroy(self):
        self.action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        # Ogni task nel dict: (goal_handle, feedback_msg) -> Task.Result
        task_number = goal_handle.request.task_number
        self.get_logger().info(f'receive goal request: {task_number}')
        feedback = Task.Feedback()
        task_fn = self.tasks.get(task_number)
        
        if task_fn is None:
            self.get_logger().error("Invalid Task Number")
            goal_handle.abort()
            out = Task.Result()
            out.success = False
            return out

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            return Task.Result()  
        
        result = task_fn(goal_handle, feedback)

        if getattr(result, "success", False):
            self.get_logger().info("Goal succeeded")
            goal_handle.succeed()
        else:
            self.get_logger().warn("Goal finished without success flag True")
        return result

def main(args=None):
    rclpy.init(args=args)
    try:
        node = TaskManager()
        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
