# GYM framework
This python package provides a framework for training reinforcement learning algorithms on the OpenManipulator-X platform.
The package consists of three distinct parts:

- custom openAI GYM environment for the lever-task.
- serial interface for the actuated lever.
- eavesdropper for extraction of manipulator internal states.

Each of these modules has its own README that supplies details about their application and internal workings.

--------------------------------------------------------------------------------

## How to begin training:
NB! This package will not work without FIRST installing the ros_packages also contained in this repository!
Please make sure that all steps of the installation guide are complete, before attempting to use the framework.

Upon release, the framework only contains one physical manipulation task: the lever_env GYM environment.
The basic usage of the framework is as follows:

```shell
$ python2
```
```python
>>> import gym

>>> import gym_environments.envs

>>> import ros_message_listener.eavesdrop as eavesdrop

>>> from embedded_lever_interface.interface import Lever as LeverClass

>>> from ManipulatorAction import ManipulatorAction

>>> action_space = ManipulatorAction()

>>> lever = LeverClass()

>>> env = gym.make('ManipulatorLeverEnv-v0', ManipulatorActions = action_space, LeverInstance = lever, goal_state=250)
```
You have now instanciated a working GYM environment, and can begin training.

NB! Remember that you NEED to have two other terminals running, for the framework to function:
```shell
$ roscore
```
```shell
$ roslaunch gym_manipulator_interface gym_controlled_manipulator.launch
```

## Developing:
If you would like to extend the framework by adding additional physical manipulation tasks, this is how you should proceed:


#### 1.1
Pick a great name for your new manipulation task environment.

#### 1.2
Make a renamed copy of the minimal GYM environment directory from manipulator_training/gym_environments/envs/minimal_GYM_env

#### 1.3
Modify the __init__.py file in manipulator_training/gym_environments/envs/ to include a register()-call for your new environment

#### 1.4 
Start filling out the minimal GYM template with the spesifics of your manipulation task.

#### 1.5 
Check out the modules/packages already available in the package. For example, if you need ROS access, have a look at using the manipulator_training/ros_message_listener module.

#### 1.6 
Create new modules/packages for your physical environment, place them in the base folder manipulator_training/, and remember to update dependencies in setup.py(if there are any).

#### 1.7 DOCUMENT YOUR WORK.

If you get stuck, have a look at how things are done for the lever_env.




### Debugging
For debugging of the ROS nodes, you can seperate the one, cloutet terminal running:
```shell
$ roslaunch gym_manipulator_interface gym_controlled_manipulator.launch
```
Into three separate terminals, running:
```shell
$ roslaunch open_manipulator_controller open_manipulator_controller.launch
``````shell
$ roslaunch open_manipulator_teleop open_manipulator_teleop_gym.launch
``````shell
$ roslaunch gym_manipulator_interface manipulator_action_server.launch
```
