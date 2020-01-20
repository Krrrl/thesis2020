#! /usr/bin/env python

import rospy

import random as rn
import time

from gym_ros_interface.srv import ActionQueue
from ManipulatorAction import ManipulatorAction

# class RandomAgent(object):

# 	def __init__(self, action_space, frequency):
# 		self.action_space = action_space
# 		#self.action_publisher = ActionPublisher(frequency)
# 		self.frequency = frequency
# 		self.request_type = "APPEND"

# 	def sampleAction(self):
# 		return self.action_space.getRandomSample()


def main():
	action_space = ManipulatorAction()

	for i in range(0, 100):
		response = action_space.queueAction(action_space.getRandomSample())
		print(response)


if __name__ == '__main__':
	main()
