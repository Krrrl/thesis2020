# thesis2020
This repo contains the code associated with my master thesis at ITK, NTNU, 2020.
The objective of the thesis was to create a framework for 


## COMPLETE SETUP
Please follow the steps provided.

### 1. Prerequisits
Install ROS kinetic and ROBOTIS' open_manipulator supplied software on a computer running Ubuntu 16.04.
Follow these two guides, to complete the aforementioned:

#### ROS kinetic & ROBOTIS package
http://wiki.ros.org/kinetic/Installation/Ubuntu
http://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/#overview

Installing ROS Kinetic(compressed install by ROBOTIS):
```shell
$ sudo apt-get update && apt-get upgrade
$ cd YOUR/PATH/ROS
$ mkdir catkin_ws
$ cd catkin_ws
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh && chmod 755 ./install_ros_kinetic.sh && bash ./install_ros_kinetic.sh
$ catkin_make
$ source devel/setup.bash
```
catkin_make command should not return any errors.

Installing ROBOTIS packages(provided by ROBOTIS):
```shell
$ sudo apt-get install ros-kinetic-ros-controllers ros-kinetic-gazebo* ros-kinetic-moveit* ros-kinetic-industrial-core
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
$ git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
$ git clone https://github.com/ROBOTIS-GIT/open_manipulator.git
$ git clone https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/open_manipulator_simulations.git
$ git clone https://github.com/ROBOTIS-GIT/robotis_manipulator.git
$ cd ~/catkin_ws && catkin_make
$ source devel/setup.bash
```
catkin_make command should not return any errors.

### 2. Clone the thesis repository
Place it at a destination of your choice, just _do not_ put the entier thing into your catkin workspace.
```shell
$ cd ~/YOUR/PATH
$ git clone https://github.com/krrrl/thesis2020
$ cd thesis2020/manipulator_training
$ sudo chmod +x setup.py
$ sudo chmod +x gym_environments/envs/lever_env/ManipulatorLeverEnv.py
```

### 3. Augmenting the ROBOTIS package
Follow the instructions of the ros_packages/README.md, namely:
- Place the gym_manipulator_interface ROS package into your catkin workspace.
- Replace ROBOTIS packge open_manipulator_msgs with the augmented version.
- Replace ROBOTIS package open_manipulator/open_manipulator_teleop with the augmented version.
- cd ~/yourpath/catkin_ws && catkin_make

```shell
$ cd ~/YOUR/PATH/thesis2020/
$ rsync -a -vh --delete ros_packages/open_manipulator_msgs/ ~/catkin_ws/src/open_manipulator_msgs/
$ rsync -a -vh --delete ros_packages/open_manipulator/open_manipulator_teleop/ ~/catkin_ws/src/open_manipulator/open_manipulator_teleop/
$ cp -a gym_manipulator_interface/ ~/catkin_ws/src/
$ cd ~/catkin_ws/ && catkin_make
$ source devel/setup.bash
```
Verify that the installation was successfull by issuing the command:
```shell
$ roscd gym_manipulator_interface
```
It should take you to the ~/catkin_ws/src/gym_manipulator_interface/ directory, if the installation was successfull.

### 4. Install manipulator_training python package
Follow the instructions of the manipulator_training/README.md, namely:
- Install the manipulator_training package to your python 2.7(yes, it is deprecated, but ROS kinetic will be sad if you dont).
- Make sure the hardware is set up as described in the README.
- Verify its installation by running the ManipulatorLeverEnv executable.



Et voila!
You may now begin training of other algorithms, by using the classic gym recepie of env = gym.make('ManipulatorLeverEnv-v0', ..)!

## Contact
Reach out if you have trouble using this, I'm happy to help!
