#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from cl_task_manager.action import Task 

class TaskManagerClient(Node):
    def __init__(self):
        super().__init__('task__manager_client')
        self.action_client = ActionClient(self, 
                                          Task, 
                                          'task_manager_server_action')  
        
        self.action_client.wait_for_server()
        self.goal = Task.Goal()
        self.goal.task_number = 1  
        
        #asynchronous goal sending
        self.future = self.action_client.send_goal_async(self.goal, 
                                           feedback_callback=self.feedback_callback)
         
        self.future.add_done_callback(self.response_callback)
        
        self.get_logger().info('Task Client Node has been started.')


    # quando inviamo l'azione
    def response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal Rejected !!')
            return
        self.get_logger().info('Goal accepted by the server, waiting for result...')
        
        self.create_timer(3.0, self.cancel_goal)
        
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)


    # quando riceviamo il feedback
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback.percentage
        self.get_logger().info('Feedback received: {0}'.format(feedback))

    # quando riceviamo il risultato finale
    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result received: {0}'.format(result.success))
        rclpy.shutdown()
        
        
    
    
def main():
    rclpy.init()
    task_client = TaskManagerClient()
    rclpy.spin(task_client)

if __name__ == '__main__':
    main()