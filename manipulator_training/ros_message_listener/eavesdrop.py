#! /usr/bin/env python

import rospy
import std_msgs

from open_manipulator_msgs.msg import KinematicsPose
from open_manipulator_msgs.msg import OpenManipulatorState

TIMEOUT = 1

def init_eavesdropper():
	rospy.init_node('eavesdropper', anonymous=True)

def get_gripper_kinematics_pose():
	kinematics_pose = rospy.wait_for_message('/open_manipulator/gripper/kinematics_pose', KinematicsPose)
	return kinematics_pose

def get_manipulator_moving_state():
	state = rospy.wait_for_message('/open_manipulator/states', OpenManipulatorState)

	if(state.open_manipulator_moving_state == '"STOPPED"'):
		return False

	elif(state.open_manipulator_moving_state == '"IS_MOVING"'):
		return True

	else:
		print("MANIPULATOR_MOVING_STATE is undefined?? The eavesdropper needs help.")


def get_topic_message(topic, msg_type):
	message = rospy.wait_for_message(topic, msg_type)
	return message

def get_available_topics_list():
	topics = rospy.get_published_topics()
	print(topics)

if __name__ == '__main__':
	print("INITIALIZING EAVESDROP...")
	init_eavesdropper()
	print("INITIALIZED!")

	print("Checking GRIPPER POSITION: ")
	data = get_gripper_kinematics_pose()
	print("Got the data!")
	print(data)

	print("--------------------")
	print("Checking MANIPULATOR_MOVING_STATE")
	data = get_manipulator_moving_state()
	print("Got the data!")
	print("IS_MOVING: {}".format(data))


	print("GENERAL FUNCTION TEST")
	print("JOINT4 COMMAND")
	print(get_topic_message("/open_manipulator/joint4_position/command", std_msgs.msg.Float64))
	print("-----------------")