import gym
import gym_environments.envs

import time

import ros_message_listener.eavesdrop as eavesdrop
from embedded_lever_interface.interface import Lever as LeverClass

from ManipulatorAction import ManipulatorAction


if __name__ == '__main__':
	print("Commencing manipulator RANDOM RESET test...")

	action_space = ManipulatorAction()

	lever = LeverClass()

	env = gym.make('ManipulatorLeverEnv-v0', ManipulatorActions = action_space, LeverInstance = lever, lever_goal_position = 250, manipulator_random_init = True, lever_random_init = True)

	env.print_gripper_kinematics()	

	env.reset()

	time.sleep(4)

	env.reset()

	time.sleep(4)

	env.reset()

	time.sleep(4)

	env.reset()
