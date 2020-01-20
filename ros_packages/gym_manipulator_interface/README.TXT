This ros package provides an interface between the ROS based OpenManipulator, and the Python base OpenAI GYM environments.


The structure is as follows:

	-The manipulator gets its trajectory from the open_manipulator_controller, which in turns gets its own input from the open_manipulator_teleop package.
	-Both the GYM environment and the OpenManipulator controller communicates to the action_server, which provides the ActionQueue service.
		The action server takes requests of either adding an action to the queue, sending the next action to be done.
			ActionQueue.request = "APPEND" || "POP"
			ActionQueue.action  = action

The chain is thus:
The GYM environment sends an "APPEND"-request to the action_server, along with its chosen action.
The action_server stores the action in the action_queue list.
The controller sends a next_in_queue-request to the action_server.
The action_server returns the next action in queue.


HOW TO USE:

Every command must be run in its own terminal!

Working with the physical platform:

-$roscore
-$roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=true
-$roslaunch open_manipulator_teleop open_manipulator_teleop_gym
-$roslaunch gym_manipulator_interface action_server.launch


Working with simulations:

-$roscore
-$roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false
-$roslaunch open_manipulator_teleop open_manipulator_teleop_gym
-$roslaunch open_manipulator_gezebo open_manipulator_gazebo.launch
-$roslaunch gym_manipulator_interface action_server.launch


