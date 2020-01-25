import gym
import gym_environments.envs

from gym import error, spaces, utils
from gym.utils import seeding

#UNCOMMENT if you intend to use the manipulator or GYM-ROS interface.
#import ros_message_listener.eavesdrop as eavesdrop
#from ManipulatorAction import ManipulatorAction

class MinimalGYM(gym.Env):

	def __init__(self):
		#self.action_space = 
		pass

	def step(self, action):
		return observation, reward, done, info

	def reset(self):
		return observation

	def render(self):
		pass

	def _get_obs(self):
		return observation

if __name__ == '__main__':
	
	#Environment unit test!
	#Cant be executed before environment is registered with gym, in envs/__init__.py
	# env = gym.make('MinimalGYM-v0'..)

	# env.step(action_space.get_random_action())

	# env.reset()