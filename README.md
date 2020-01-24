# thesis2020
This repo contains the code associated with my master thesis at ITK, NTNU, 2020.
The objective of the thesis was to create a framework for 


## COMPLETE FRAMEWORK INSTALLATION
Please follow the steps provided.

### 1. Prerequisits
Install ROS kinetic and ROBOTIS' open_manipulator supplied software on a computer running Ubuntu 16.04.
Follow these two guides, to complete the aforementioned:

#### ROS kinetic & ROBOTIS package
The installation of ROS kinetic and the ROBOTIS packages can be read more about on their respective pages:
http://wiki.ros.org/kinetic/Installation/Ubuntu
http://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/#overview

Installing ROS Kinetic(compressed install by ROBOTIS):
```shell
$ sudo apt-get update && apt-get upgrade
$ cd YOUR/PATH/ROS
$ mkdir catkin_ws
$ cd catkin_ws
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh && chmod 755 ./install_ros_kinetic.sh && bash ./install_ros_kinetic.sh
```
Please reboot your computer before proceeding to the next step.

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

##### Changing latency setting
The default latency of serial communication on Ubuntu 16.04 is 16 ms.
To change it to 1 ms, do the following:
Open a separate terminal for the roscore:
```shell
$ roscore
```
Then open another terminal, executing the command:
```shell
$ rosrun open_manipulator_controller create_udev_rules
```
The last command will prompt a sudo login.


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
To augment the ROBOTIS package, we will need to:
- Place the gym_manipulator_interface ROS package into the catkin workspace.
- Replace ROBOTIS packge open_manipulator_msgs with the augmented version.
- Replace ROBOTIS package open_manipulator/open_manipulator_teleop with the augmented version.

```shell
$ cd ~/YOUR/PATH/thesis2020/ros_packages/
$ mv ros_packages/gym-manipulator-interface.egg-link /usr/local/lib/python2.7/dist-packages/
$ mv gym_manipulator_interface/ ~/catkin_ws/src/
$ rsync -a -vh --delete open_manipulator_msgs/ ~/catkin_ws/src/open_manipulator_msgs/
$ rsync -a -vh --delete open_manipulator/open_manipulator_teleop/ ~/catkin_ws/src/open_manipulator/open_manipulator_teleop/
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
- Install the manipulator_training package
- Make sure the hardware is set up as described in the README.
- Verify its installation by running the ManipulatorLeverEnv executable.

```shell
$ cd ~/YOUR/PATH/manipulator_training
$ pip install --user -e .
```
The last command will start a download of the required packages, before stating that it "successfully installed manipulator_training".

Now, you should also check and make sure that the hardware is connected properly:
-Manipulator connected according to <http://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_setup/#connection>
-Lever: 230VAC power outlet and PC <USB> uStepper. NB! Power up with the lever in the _UPRIGHT_ position!

The manipulator should always be placed in its prefered starting position before use.
Please see have a look at the picture provided at: <http://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_controller_package/#launch-controller>

#### Verification of installation
To verify that the entire installation is complete, we will execute the framework's integration test.
In three separate terminals, run the commands:

```shell
$ roscore
```

```shell
$ roslaunch gym_manipulator_interface gym_controlled_manipulator.launch
```

```shell
$ cd ~/YOUR/PATH/thesis2020/manipulator_training/gym_environments/envs/lever_env/
$ python ManipulatorLeverEnv.py
```
This should cause both the lever and manipulator to move, settling at their respective reset positions.

#### Et voila!
You may now begin training of other algorithms, by using the classic gym recepie of env = gym.make('ManipulatorLeverEnv-v0', ..)! 
Please see the manipulator_training/README.md for more detail on how to utilize the framwork.

## Contact
Reach out if you have trouble using this, I'm happy to help!
