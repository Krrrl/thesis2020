from gym.envs.registration import register

register(id='ManipulatorLeverEnv-v0',
		entry_point='gym_environments.envs.lever_env:ManipulatorLeverEnv'
)
