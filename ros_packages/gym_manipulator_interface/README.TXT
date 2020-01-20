This ros package provides an interface between the ROS based OpenManipulator, and the Python base OpenAI GYM environments.

The main structure is as follows:

	-The manipulator gets its control inputs from the open_manipulator_controller, which in turns gets its own input from the open_manipulator_teleop package.
	-Both the GYM environment and the OpenManipulator controller communicates to the action_server, which provides the ActionQueue service.
		The action server takes requests of either adding an action to the queue, sending the next action to be done.
			ActionQueue.request = "APPEND" || "POP"
			ActionQueue.action  = action

The chain is thus:
The GYM environment sends an "APPEND"-request to the action_server, along with its chosen action.
The action_server stores the action in the action_queue list.
The controller sends a next_in_queue-request to the action_server.
The action_server returns the next action in queue.
The controller converts the action to a control input for the manipulator.
The manipulator executes the action.

All actions are executed as fast as possible by the manipulator.
All created packages contains apt README files for more detail. If detail is insufficient, dont hesitate to open an issue or contact the package maintainer.



HOW TO USE:


--------------------------------------------------------------------------------------------
FIRST: Make sure that the hardware is connected as follows:
	- Poweroutlet for MANIPULATOR connected
		-The powerbutton on the MANIPULATOR power circuit needs to be in the "dot" position.
	- Poweroutlet for LEVER connected
	- PC < USB > U2D2(MANIPULATOR blinking circuit)
	- PC < USB > uStepper(LEVER circuit)
--------------------------------------------------------------------------------------------


NB! Do not forget to run "$roscore" in a separate terminal!

Short version, clouded terminal:

	$roslaunch gym_manipulator_interface gym_controlled_manipulator.launch


Long version:

	If you want to take the scenic route then follow on. From here, every command must be run in its own terminal!

	Working with the physical platform:

		$roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=true
		$roslaunch open_manipulator_teleop open_manipulator_teleop_gym
		$roslaunch gym_manipulator_interface action_server.launch


	Working with simulations:

		$roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false
		$roslaunch open_manipulator_teleop open_manipulator_teleop_gym
		$roslaunch open_manipulator_gezebo open_manipulator_gazebo.launch
		$roslaunch gym_manipulator_interface action_server.launch

NB! This in it self only readies the manipulator for inputs from the GYM environments.

You can verify that the setup is correct and working by running the integrations test of the ManipulatorLeverEnv()-class(for example: 

	$python YOUR/PATH/manipulator_training/gym_environments/envs/lever_env/ManipulatorLeverEnv.py

This will verify both MANIPULATOR and LEVER, along with their interfaces.
