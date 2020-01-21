# thesis2020
This repo contains the code associated with my master thesis at ITK, NTNU, 2020.
The objective of the thesis was to create a framework for 


## SETUP
Please follow the steps provided.

### 1. Prerequisits
Install ROS kinetic and ROBOTIS' open_manipulator supplied software on a computer running Ubuntu 16.04.
Follow these two guides, to complete the aforementioned:

#### ROS kinetic
http://wiki.ros.org/kinetic/Installation/Ubuntu
Choose the full-desktop install, preferably(the regular desktop install works too).

#### ROBOTIS package
http://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/#overview
Complete step 5 and 6.

### 2. Clone this repository
To a destination of your choice, just _do not_ put the entier thing into your catkin workspace.

### 3. Augment the ROBOTIS package
Follow the instructions of the ros_packages/README.md, namely:
- Place the gym_manipulator_interface ROS package into your catkin workspace.
- Replace ROBOTIS packge open_manipulator_msgs with the augmented version.
- Replace ROBOTIS package open_manipulator/open_manipulator_teleop with the augmented version.
- cd ~/yourpath/catkin_ws && catkin_make

### 4. Install manipulator_training python package
Follow the instructions of the manipulator_training/README.md, namely:
- Install the manipulator_training package to your python 2.7(yes, it is deprecated, but ROS kinetic will be sad if you dont).
- Make sure the hardware is set up as described in the README.
- Verify its installation by running the ManipulatorLeverEnv executable.

Et voila!
You may now begin training of other algorithms, by using the classic gym recepie of env = gym.make('ManipulatorLeverEnv-v0', ..)!

## Contact
Reach out if you have trouble using this, I'm happy to help!
