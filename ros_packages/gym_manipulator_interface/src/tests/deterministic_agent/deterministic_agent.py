#! /usr/bin/env python

import rospy

import random as rn
import time

from gym_ros_interface.srv import ActionQueue
from ManipulatorAction import ManipulatorAction

class DeterministicAgent(object):

	def __init__(self, deterministic_action):
		self.request_type = "APPEND"
		self.deterministic_action = deterministic_action

	def sampleAction(self):
		return self.deterministic_action


def main():
	action_space = ManipulatorAction()

	static_action = 'y'
	Agent = DeterministicAgent(static_action)

	for i in range(0, 100):
		response = action_space.queueAction(Agent.sampleAction())
		print(response)


if __name__ == '__main__':
	main()
