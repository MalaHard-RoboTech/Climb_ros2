
import time


def percentage_complete(i):
        return i * 10
    
def task_0(feedback_msg, goal_handle):
    for i in range(10):
        feedback_msg.percentage = i * 10
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(0.5)
        


