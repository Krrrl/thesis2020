This python package provides a framework for training reinforcement learning algorithms on the OpenManipulator-X platform.
The package consists of three distinct parts:

-custom openAI GYM environment for the lever-task.
-serial interface for the actuated lever.
-eavesdropper for extraction of manipulator internal states.

Each of these modules has its own README that supplies details about their application and internal workings.





How to use:
--------------------------------------------------------------------------------
Given that you have FIRST installed the ROS package "rl_manipulator_lever_env".
See its README for details on how to install it.
--------------------------------------------------------------------------------
/manipulator_lever_task_environment$ pip install --user -e .

$ python

>>> import gym
>>> import custom_gym_env.envs
>>> import manipulator_state_listener.eavesdrop as eavesdrop
>>> from lever_interface.interface import Lever as LeverClass
>>> from ManipulatorAction import ManipulatorAction

>>> action_space = ManipulatorAction()
>>> lever = LeverClass()

>>> env = gym.make('ManipulatorLeverEnv-v0', ManipulatorActions = action_space, LeverInstance = lever, goal_state=250)

After this, you can call any of the ManipulatorLeverEnv class methods.

>>> env.step(action)

>>> env.reset()
etc.

--------------------------------------------------------------------------------

------------------------------
------------------------------

Alternatively, the following is the full proceedure for the complete integrations test of GYM environment + lever + open_manipulator.
Each command runs from a seperate terminal.
NB! Make sure that the lever is in the upright position, and the manipulator is in its recommended starting position(resting on only the claw).

>>> roscore

>>> roslaunch open_manipulator_controller open_manipulator_controller.launch

>>> roslaunch gym_ros_interface action_server.launch

>>> roslaunch open_manipulator_teleop open_manipulator_teleop_agent

>>> python ManipulatorLeverEnv.py

The result of the last command will cause both manipulator and lever to move. The lever resets trice(a unit-test of the lever interface is performed), first to its end-position and then to a random position, before coming to rest at the endpoint positon.
The manipulator gets a total of 17 inputs to increase its second joint angle, causing it to tilt upwards, about a quarter of its full range.
