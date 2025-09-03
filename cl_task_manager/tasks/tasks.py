import time
from cl_task_manager.action import Task  # per creare il Result finale

def percentage_complete(i):
    return i * 10

def task_0(node, goal_handle, feedback_msg):
    for i in range(10):
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = Task.Result()
            result.success = False
            node.get_logger().warn('[task_0] Canceled')
            return result
        feedback_msg.percentage = percentage_complete(i)
        node.get_logger().info(f'[task_0] progress: {feedback_msg.percentage}%')
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(0.5)
    
    feedback_msg.percentage = 100
    goal_handle.publish_feedback(feedback_msg)
    result = Task.Result()
    result.success = True
    node.get_logger().info('[task_0] Completed: 100%')
    return result