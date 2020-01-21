# ROS code

This folder contains all necessary ros code.
Its divided into three separate directories.


### gym_manipulator_interface
gym_manipulator_interface is a stand alone package, and needs to be moved into your ~/catkin_ws/src/ folder.


### open_manipulator/open_manipulator_teleop
open_manipulator/open_manipulator_teleop is an augmented version of the publicly available open_manipulator_teleop package.
The augmentation allows gym to assume control of the manipulator directly.
The augmented version should replace your official version.

### open_manipulator_msgs
open_manipulator_msgs is also augmented, and should replace the official version.
The augemntation is just the addition of the action_queue service from gym_manipulator_interface.


#### CHANGELOG
Please see the CHANGELOG.md for an exact changes/amendments done to the supplied open_manipulator ros packages.
