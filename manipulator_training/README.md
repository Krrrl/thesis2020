# GYM framework
This python package provides a framework for training reinforcement learning algorithms on the OpenManipulator-X platform.
The package consists of three distinct parts:

- custom openAI GYM environment for the lever-task.
- serial interface for the actuated lever.
- eavesdropper for extraction of manipulator internal states.

Each of these modules has its own README that supplies details about their application and internal workings.


# How to use:
--------------------------------------------------------------------------------
Given that you have FIRST installed the ROS package "gym_manipulator_interface"(also provided in this repository).
See its README for details on how to install it.
--------------------------------------------------------------------------------
YOUR/PATH/manipulator_training$ python setup.py install --user
(alternatively, use the command "$ pip install --user -e . " if you are activly developing on the framework).

$ python

&gt;&gt;&gt; import gym

&gt;&gt;&gt; import gym_environments.envs

&gt;&gt;&gt; import ros_message_listener.eavesdrop as eavesdrop

&gt;&gt;&gt; from embedded_lever_interface.interface import Lever as LeverClass

&gt;&gt;&gt; from ManipulatorAction import ManipulatorAction

&gt;&gt;&gt; action_space = ManipulatorAction()

&gt;&gt;&gt; lever = LeverClass()

&gt;&gt;&gt; env = gym.make('ManipulatorLeverEnv-v0', ManipulatorActions = action_space, LeverInstance = lever, goal_state=250)

After this, you can call any of the ManipulatorLeverEnv class methods.

&gt;&gt;&gt; env.step(action)

&gt;&gt;&gt; env.reset()

etc.

--------------------------------------------------------------------------------

Alternatively, the following is the full proceedure for the complete integrations test of GYM environment + lever + OpenManipulator.
Each command runs from a seperate terminal.
NB! Make sure that the lever is in the upright position, and the manipulator is in its recommended starting position(resting on only the claw).

$ roscore

$ roslaunch open_manipulator_controller open_manipulator_controller.launch

$ roslaunch gym_ros_interface action_server.launch

$ roslaunch open_manipulator_teleop open_manipulator_teleop_agent

$ python ManipulatorLeverEnv.py

The result of the last command will cause both manipulator and lever to move. The lever resets trice(a unit-test of the lever interface is performed), first to its end-position and then to a random position, before coming to rest at the endpoint positon.
The manipulator gets a total of 17 inputs to increase its second joint angle, causing it to tilt upwards, about a quarter of its full range.

--------------------------------------------------------------------------------

For convinience, there exists a shorthand version of the aforementioned:

$ roscore

$ roslaunch gym_manipulator_interface gym_controlled_manipulator.launch

And then whatever python program you want to be executed.
