# ros_message_listener
This package contains the interface between the internal ROS TOPICS and any python module.

The primary use of this is to extract messages from ROS topics to the OpenAI GYM environments, using this "eavesdrop.py" module.

The package also contains legacy code, primarily for completeness and future reference on what not to do.
Specifically, this is the gripper_position_listener.py, which uses global variables and in general is an ugly mess.


eavesdrop.py 
If you decide to add additional functionality, please document it in this README.


## IMPORTANT:
The module making use of the eavesdropper MUST call the init_eavesdropper function ONCE, before any other function calls.


## How to use:
The eavesdrop module has some topic-spesific functions implemented(namely for the OpenManipulator state topics), but it also provides a general function to extract messages.

&gt;&gt;&gt; import eavesdrop

&gt;&gt;&gt; init_eavesdropper()

&gt;&gt;&gt; pose = eavesdrop.get_gripper_kinematics_pose()

&gt;&gt;&gt; moving_state = eavesdrop.get_manipulator_moving_state()

#### General case:

&gt;&gt;&gt; msg = get_topic_message(topic_name -> str, msg_type)

The general case function can be used to extract any message from any topic, as long as you know the topic name and message type. Both topic name and message type can be found by using "rqt" with the specific ros nodes running.
```shell
$ rqt 
```




