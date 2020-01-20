
import rospy

import random as rn
import time

from gym_ros_interface.srv import ActionQueue

class ManipulatorAction():

	# Action space as follows:
	# Joint space:
	# 	y / h - joint 1 increase / decrease
	# 	u / j - joint 2 increase / decrease
	# 	i / k - joint 3 increase / decrease
	# 	o / l - joint 4 increase / decrease
	# 	g / f - gripper open / gripper close
	#
	#	'1': init pose
	#	'2': home pose
	# NB! No implementation of task space yet, due to issue with freezing of the open manipulator controller.

	def __init__(self):
		self.request_type = "APPEND"
		self.action_space = {0:'y', 
								1:'h',
								2:'u',
								3:'j',
								4:'i',
								5:'k',
								6:'o',
								7:'l',
								8:'g',
								9:'f',
								'init':'1',
								'home':'2'}

	def getSize(self):
		return len(self.action_space)

	def getAction_str(self, action):
		return self.action_space.get(action, "Invalid action")

	def getRandomSample(self):
		choice = rn.sample(self.action_space, 1)
		return self.getAction_str(choice.pop())

	def printActionSpace(self):
		for key, value in self.action_space.items():
			print(key, value)

	def validAction(self, action):
		if(action in self.action_space):
			return True
		else:
			return False

	def queueAction(self, action):
		if(type(action) is int):
			action = self.getAction_str(action)

		rospy.wait_for_service('/open_manipulator/queue_action')

		try:
			queue_action = rospy.ServiceProxy('/open_manipulator/queue_action', ActionQueue)
			response = queue_action(self.request_type, action)
			return response

		except rospy.ServiceException, e:
			print("Service call failed! %s" %e)
