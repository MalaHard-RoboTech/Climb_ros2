#cl_task_manager

This package enables high-level management of the various tasks the robot can execute.

All available tasks are defined in the  `task.py` file.
The `task_manager.py` file handles these tasks using ROS 2 actions, allowing them to be managed asynchronously.

Thanks to this structure, commands can also be sent from external interfaces, such as a web application or another ROS 2 node.