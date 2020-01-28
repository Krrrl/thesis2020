* contemplate implementing intermidiate rewards for all changes in lever_position, not just reaching terminal state.
  This can be done by just getting half the negative reward for actions that lead to change in lever position, regardless of its direction. This would make the area around the lever an area of interest, for sure.

* Add camera data to manipulator observations. For tips on how to do this you can check the master thesis, where I added camera data to a GYM environment). The camera data needs processing though, so you might need to add opencv into this mix.

* Make the entire "OpenAI Baseline" library of reinforcement learning methods available to train on the manipulator.
This is currently not possible, because the manipulator software runs on ROS Kinetic, which is strictly python 2.7 - which Baselines does not support!
Luckily, the fix for this is pretty easy, since the only problem here is the import of rospy, and thus we just need to:
.* Make the gym_manipulator_interface/src/ManipulatorAction.py get its input actions from a FILE, rather than a function call.
I.e queueAction() checks for entries in a target file set up initialization.
.* This means that ManipulatorLeverEnv needs to write its chosen actions to that same file, instead of just calling the ManipulatorAction queueAction() function.
.* This also means that the manipulator_training/ros_package_listener also need to be upgraded in the same manner(eavesdropper runs on its own and writes to a file with a certain frequency, and then the GYM environment reads from this file).


* Rename the ManipulatorLeverEnv task into a general ManipulatorTrainingEnv that takes the manipulation task as an argument to its GYM environment instantiation(like it already does). This should be done to easily accommodate swapping between different manipulation tasks in the future. This also means that the task-functionality that is required for the manipulator training is the same for all manipulation tasks implemented in the future. 
Example: 
  ```python
  env = gym.make('ManipulatorTraining-v0', task = Lever(goal_state=250, ..) ..)
  ..
  env.__init__(): .. self.task = task ..
  env.step(action): .. reward = task.isTerminal() ..
  ```
  Could be very cool.

* The alternative to the previously suggested is to clean up the code, and implement the specific task functions in copies of the ManipulatorLeverEnv as it currently is:
  ..* Make ManipulatorLeverEnv inherit from LeverInterface, rather than getting passed an instance.
