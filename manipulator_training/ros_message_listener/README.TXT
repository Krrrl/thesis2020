HOW TO USE eavesdrop.py:

MINIMUM:
	import eavesdrop

	init_eavesdropper()

OPTIONAL:
	pose = eavesdrop.get_gripper_kinematics_pose()

	moving_state = eavesdrop.get_manipulator_moving_state()




SUMMARY:

This module contains the interface between the manipulators internal ROS TOPICS and any python module.

The primary use of this is to extract messages from ROS topics to the OpenAI GYM environments, using this "eavesdrop.py" module.
The gripper_position_listener.py module and its CLASS TESTER script is legacy, but included for completeness and future reference.

It is not recommended to use the gripper_position_listener class, since it uses global variables and generally is an ugly mess.


If you decide to add additional functionality, please document it here.
eavesdrop.py can easily be augmented to collect messages from new topics, should there be any other of interest, just follow the function boilerplate:

def get_something:
	data = rospy.wait_for_message("/YOUR_TOPIC", msg_type)
	return data

IMPORTANT:
The module making use of the eavesdropper MUST call the init_eavesdropper function ONCE, before any other function calls.


