#! /usr/bin/env python

import rospy

from open_manipulator_msgs.msg import KinematicsPose

current_gripper_position = None

def get_gripper_position():
	global current_gripper_position
	return current_gripper_position

def set_gripper_position(data):
	global current_gripper_position
	current_gripper_position = data

class gripper_state_listener():

	def __init__(self, frequency):
		pass

	def update_position(self, data):
		set_gripper_position(data)	

	def run(self):
		rospy.init_node("gripper_position_listener", anonymous=True)

		rospy.Subscriber("/open_manipulator/gripper/kinematics_pose", KinematicsPose, self.update_position)

		rospy.spin()


if __name__ == '__main__':
	#rospy.init_node("gripper_state_listener", anonymous=True)
	gripper_listener = gripper_state_listener(10)
	gripper_listener.run()


