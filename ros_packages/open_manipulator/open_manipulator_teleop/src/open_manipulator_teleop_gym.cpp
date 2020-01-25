/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#include "open_manipulator_teleop/open_manipulator_teleop_gym.h"

OpenManipulatorTeleop::OpenManipulatorTeleop()
    :node_handle_(""),
     priv_node_handle_("~")
{
  present_joint_angle_.resize(NUM_OF_JOINT);
  present_kinematic_position_.resize(3);

  initClient();
  initSubscriber();

  ROS_INFO("OpenManipulator initialization");
}

OpenManipulatorTeleop::~OpenManipulatorTeleop()
{
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
}

void OpenManipulatorTeleop::initClient()
{
  goal_joint_space_path_from_present_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path_from_present");
  goal_task_space_path_from_present_position_only_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_from_present_position_only");

  goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
  goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");

  //OWN ADDITION
  queue_action_client_ = node_handle_.serviceClient<open_manipulator_msgs::ActionQueue>("queue_action");

}
void OpenManipulatorTeleop::initSubscriber()
{
  joint_states_sub_ = node_handle_.subscribe("joint_states", 10, &OpenManipulatorTeleop::jointStatesCallback, this);
  kinematics_pose_sub_ = node_handle_.subscribe("kinematics_pose", 10, &OpenManipulatorTeleop::kinematicsPoseCallback, this);
}

void OpenManipulatorTeleop::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);
  for(std::vector<int>::size_type i = 0; i < msg->name.size(); i ++)
  {
    if(!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;

}

void OpenManipulatorTeleop::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);
  present_kinematic_position_ = temp_position;
}

std::vector<double> OpenManipulatorTeleop::getPresentJointAngle()
{
  return present_joint_angle_;
}
std::vector<double> OpenManipulatorTeleop::getPresentKinematicsPose()
{
  return present_kinematic_position_;
}

bool OpenManipulatorTeleop::setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if(goal_joint_space_path_from_present_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if(goal_joint_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setToolControl(std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name.push_back(priv_node_handle_.param<std::string>("end_effector_name", "gripper"));
  srv.request.joint_position.position = joint_angle;

  if(goal_tool_control_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;
  srv.request.planning_group = priv_node_handle_.param<std::string>("end_effector_name", "gripper");
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);
  srv.request.path_time = path_time;

  if(goal_task_space_path_from_present_position_only_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

void OpenManipulatorTeleop::printText()
{
  printf("---------------------------\n");
  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
         getPresentJointAngle().at(0),
         getPresentJointAngle().at(1),
         getPresentJointAngle().at(2),
         getPresentJointAngle().at(3));
  printf("Present Kinematics Position X: %.3lf Y: %.3lf Z: %.3lf\n",
         getPresentKinematicsPose().at(0),
         getPresentKinematicsPose().at(1),
         getPresentKinematicsPose().at(2));
  printf("---------------------------\n");
}

void OpenManipulatorTeleop::setGoal(char ch)
{
  std::vector<double> goalPose;  goalPose.resize(3, 0.0);
  std::vector<double> goalJoint; goalJoint.resize(NUM_OF_JOINT, 0.0);

  if(ch == 'w' || ch == 'W')
  {
    printf("input : w \tincrease(++) x axis in task space\n");
    goalPose.at(0) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(ch == 's' || ch == 'S')
  {
    printf("input : s \tdecrease(--) x axis in task space\n");
    goalPose.at(0) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(ch == 'a' || ch == 'A')
  {
    printf("input : a \tincrease(++) y axis in task space\n");
    goalPose.at(1) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(ch == 'd' || ch == 'D')
  {
    printf("input : d \tdecrease(--) y axis in task space\n");
    goalPose.at(1) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(ch == 'z' || ch == 'Z')
  {
    printf("input : z \tincrease(++) z axis in task space\n");
    goalPose.at(2) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(ch == 'x' || ch == 'X')
  {
    printf("input : x \tdecrease(--) z axis in task space\n");
    goalPose.at(2) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(ch == 'y' || ch == 'Y')
  {
    printf("input : y \tincrease(++) joint 1 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1"); goalJoint.at(0) = JOINT_DELTA;
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if(ch == 'h' || ch == 'H')
  {
    printf("input : h \tdecrease(--) joint 1 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1"); goalJoint.at(0) = -JOINT_DELTA;
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }

  else if(ch == 'u' || ch == 'U')
  {
    printf("input : u \tincrease(++) joint 2 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2"); goalJoint.at(1) = JOINT_DELTA;
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if(ch == 'j' || ch == 'J')
  {
    printf("input : j \tdecrease(--) joint 2 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2"); goalJoint.at(1) = -JOINT_DELTA;
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }

  else if(ch == 'i' || ch == 'I')
  {
    printf("input : i \tincrease(++) joint 3 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3"); goalJoint.at(2) = JOINT_DELTA;
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if(ch == 'k' || ch == 'K')
  {
    printf("input : k \tdecrease(--) joint 3 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3"); goalJoint.at(2) = -JOINT_DELTA;
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }

  else if(ch == 'o' || ch == 'O')
  {
    printf("input : o \tincrease(++) joint 4 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4"); goalJoint.at(3) = JOINT_DELTA;
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if(ch == 'l' || ch == 'L')
  {
    printf("input : l \tdecrease(--) joint 4 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4"); goalJoint.at(3) = -JOINT_DELTA;
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }

  else if(ch == 'g' || ch == 'G')
  {
    printf("input : g \topen gripper\n");
    std::vector<double> joint_angle;

    joint_angle.push_back(0.01);
    setToolControl(joint_angle);
  }
  else if(ch == 'f' || ch == 'F')
  {
    printf("input : f \tclose gripper\n");
    std::vector<double> joint_angle;
    joint_angle.push_back(-0.01);
    setToolControl(joint_angle);
  }

  else if(ch == 'm' || ch == 'M')
  {
    printf("input : m \thome pose\n");
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 3.0;

    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(-1.05);
    joint_name.push_back("joint3"); joint_angle.push_back(0.35);
    joint_name.push_back("joint4"); joint_angle.push_back(0.70);
    setJointSpacePath(joint_name, joint_angle, path_time);

    //open gripper
    std::vector<double> gripper_angle;
    gripper_angle.push_back(0.01);
    setToolControl(gripper_angle);
  }
  else if(ch == 'n' || ch == 'N')
  {
    printf("input : n \tresetting to IDEAL start pose\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 3.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.73);
    joint_name.push_back("joint2"); joint_angle.push_back(0.01);
    joint_name.push_back("joint3"); joint_angle.push_back(0.42);
    joint_name.push_back("joint4"); joint_angle.push_back(1.11);
    setJointSpacePath(joint_name, joint_angle, path_time);

    //close gripper
    std::vector<double> gripper_angle;
    gripper_angle.push_back(-0.01);
    setToolControl(gripper_angle);
  }
  else if(ch == 'p' || ch == 'p')
  {
    printf("input : p \tresetting to RANDOM start pose\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 3.0;

    float random_positions[NUM_OF_JOINT] = {0};
    getRandomManipulatorPosition(random_positions);

    joint_name.push_back("joint1"); joint_angle.push_back(random_positions[0]);
    joint_name.push_back("joint2"); joint_angle.push_back(random_positions[1]);
    joint_name.push_back("joint3"); joint_angle.push_back(random_positions[2]);
    joint_name.push_back("joint4"); joint_angle.push_back(random_positions[3]);
    setJointSpacePath(joint_name, joint_angle, path_time);

    //close gripper
    std::vector<double> gripper_angle;
    gripper_angle.push_back(-0.01);
    setToolControl(gripper_angle);
  }
}

void OpenManipulatorTeleop::restoreTerminalSettings(void)
{
    tcsetattr(0, TCSANOW, &oldt_);  /* Apply saved settings */
}

void OpenManipulatorTeleop::disableWaitingForEnter(void)
{
  struct termios newt;

  tcgetattr(0, &oldt_);  /* Save terminal settings */
  newt = oldt_;  /* Init new settings */
  newt.c_lflag &= ~(ICANON | ECHO);  /* Change settings */
  tcsetattr(0, TCSANOW, &newt);  /* Apply settings */
}


//BEGINING OF INSERTED FUNCTIONS

void OpenManipulatorTeleop::getRandomManipulatorPosition(float* random_pos_array)
{
  float joint_limits[] = {-3.10, 3.10, -1.31, -0.31, -1.45, 0.95, -1.7, -0.04};

  //JOINT HARD LIMITATIONS
  //Left for future reference:
  //float joint1_MIN = -3.10;
  //float joint1_MAX = 3.10;

  //float joint2_MIN = -1.95;
  //float joint2_MAX = 1.57;
  
  //float joint3_MIN = -1.57;
  //float joint3_MAX = 1.43;
 
  //float joint4_MIN = -1.7;
  //float joint4_MAX = 1.9;
  
  float lever_protection_sector_high = 0.34;
  float lever_protection_sector_low = -0.94;

  for (int i = 0; i < NUM_OF_JOINT; ++i)
  {
    random_pos_array[i] = joint_limits[(i*2)] + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(joint_limits[(i*2)+1]-joint_limits[(i*2)])));
  }

  //Horrible, but effective.
  //The following protects the manipulator and lever from colliding during random initialization.
  //I welcome a prettier solution to this.
  while((lever_protection_sector_low < (random_pos_array[0])) and ((random_pos_array[0] < lever_protection_sector_high)))
  {
    random_pos_array[0] = joint_limits[0] + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(joint_limits[1]-joint_limits[0])));
  }

  //Solution for rounding to two decimal place.
  for (int i = 0; i < NUM_OF_JOINT; ++i)
  {
    random_pos_array[i] = (float)((int)(random_pos_array[i]*100 + 0.5))/100;
  }
}

char OpenManipulatorTeleop::getChar(void)
{  
  open_manipulator_msgs::ActionQueue srv;
  srv.request.request_type = REQUEST_ACTION;
  srv.request.action = "NONE";

  if(queue_action_client_.call(srv))
  {
    ROS_INFO("Server called successfully, provided action: %s \n", srv.response.action);
    char returned_action[2];
    std::strcpy(returned_action, srv.response.action.c_str());
    std::cout << "This is the provided action: " << std::endl;
    std::cout << returned_action[0] << std::endl;
    return returned_action[0];
  }
  
  else
  {
    std::cout << "UNSUCCESSFUL attempt to contact action server." << std::endl;
    return '!';
  }
}

//END OF INSERTED FUNCTIONS

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "open_manipulator_teleop");

  OpenManipulatorTeleop openManipulatorTeleop;

  //while the
  std::srand(time(NULL));

  ROS_INFO("OpenManipulator teleoperation using GYM environment start.");
  openManipulatorTeleop.disableWaitingForEnter();

  ros::spinOnce();
  openManipulatorTeleop.printText();

  char ch;
  while (ros::ok() && ((ch = openManipulatorTeleop.getChar()) != 'q')) 
  {
    if (ch != '!')
    {
      ros::spinOnce();
      openManipulatorTeleop.printText();
      ros::spinOnce();
      openManipulatorTeleop.setGoal(ch); 
    }
    else
    {
      ros::spinOnce();
    }
    ros::Duration(0, 500000000).sleep();
  }

  printf("input : q \tTeleop. is finished\n");
  openManipulatorTeleop.restoreTerminalSettings();

  return 0;
}
