import gym

import custom_gym_env.envs

print("IMPORTED!")

env = gym.make('ManipulatorLeverEnv-v0')

env.step()

print("DONE!")
