import gym
import gym_environments.envs

from gym import error, spaces, utils
from gym.utils import seeding


import time

import ros_message_listener.eavesdrop as eavesdrop
from embedded_lever_interface.interface import Lever as LeverClass

from ManipulatorAction import ManipulatorAction

class ManipulatorLeverEnv(gym.Env):

  metadata = {'render.modes': ['human']}

  def __init__(self, ManipulatorActions, LeverInstance, goal_state=0, use_platform=False):

    self.ManipulatorActions = ManipulatorActions
    self.Lever = LeverInstance

    self.__SLACK = 0.001
    self.__use_platform = use_platform

    eavesdrop.init_eavesdropper()

    self.previous_gripper_kinematics = self.eavesdrop_current_gripper_state()
    self.current_gripper_kinematics = self.eavesdrop_current_gripper_state()

    #TASK PARAMS
    self.goal_state = goal_state

    self.Lever.set_Goal(self.goal_state)


  def step(self, action):
    if(self.ManipulatorActions.validAction(action)):
      self.previous_gripper_kinematics = self.current_gripper_kinematics

      self.ManipulatorActions.queueAction(action)
      
      self.wait_for_action_execution()

      self.current_gripper_kinematics = self.eavesdrop_current_gripper_state()

    else:
      print("---------------------------------")
      print("-------- INVALID ACTION ---------")
      print("---------------------------------")
      


  def reset(self, manipulator_random_init = False, lever_random_init = False):

    print("---------------------------------------")
    print("---------RESETTING ENVIRONMENT---------")
    print("---------------------------------------")

    #RESETTING MANIPULATOR
    if(manipulator_random_init):
      print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
      print("!!!!!!!!!! NOT YET IMPLEMENTED !!!!!!!!!!")
      print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

    else:
      self.ManipulatorActions.queueAction(10)
      time.sleep(2)
      self.ManipulatorActions.queueAction(11)
      time.sleep(2)

    #RESETTING LEVER
    if(lever_random_init):
      self.Lever.reset_RAN()
      time.sleep(2)
    else:
      self.Lever.reset_ABS()
      time.sleep(2)

    #This is ment to be blocking, so that the environment has time to reset before new querries. 
    lever_ready = False
    manipulator_moving = True
    while(not(lever_ready and (not manipulator_moving))):
      lever_ready = self.Lever.env_Ready()
      manipulator_moving = self.eavesdrop_manipulator_moving_state()
      print("Lever ready: {}".format(lever_ready))
      print("Manipulator moving: {}".format(manipulator_moving))
      print("... waiting for reset ...")
      time.sleep(1)

    
    self.previous_gripper_kinematics = self.eavesdrop_current_gripper_state()
    self.current_gripper_kinematics = self.eavesdrop_current_gripper_state()

    return self.current_gripper_kinematics


  def render(self, mode='human', close = False):
    print("No rendering. Its a physical system.")

  def update_gripper_kinematics(self):
    self.current_gripper_kinematics = self.eavesdrop_current_gripper_state()
    print("Updated current gripper kinematics to: ")
    print(self.current_gripper_kinematics)

  def print_gripper_kinematics(self):
      print("---------------------------------------")
      print("PREVIOUS GRIPPER STATE: ")
      print(self.previous_gripper_kinematics)
      print("---------------------------------------")
      print("CURRENT GRIPPER STATE: ")
      print(self.current_gripper_kinematics)
      print("---------------------------------------")

  def wait_for_action_execution(self):
    #BLOCKING!
    if(self.__use_platform):
      while self.eavesdrop_manipulator_moving_state():
        time.sleep(0.1)
        continue
    elif(not self.__use_platform):
      while(not self.gripper_position_changed(self.previous_gripper_kinematics, self.current_gripper_kinematics)):
        self.current_gripper_kinematics = self.eavesdrop_current_gripper_state()
        time.sleep(0.1)

  def gripper_position_changed(self, previous_pos, current_pos):
    # print("I WAS ASKED TO COMPARE, PREVIOUS POS: ")
    # print(previous_pos)
    # print("AND CURRENT POS: ")
    # print(current_pos)

    if(((current_pos.pose.position.x <= (previous_pos.pose.position.x - self.__SLACK)) or (current_pos.pose.position.x > (previous_pos.pose.position.x + self.__SLACK))) 
        or ((current_pos.pose.position.y <= (previous_pos.pose.position.y - self.__SLACK)) or (current_pos.pose.position.y > (previous_pos.pose.position.y + self.__SLACK)))
        or ((current_pos.pose.position.z <= (previous_pos.pose.position.z - self.__SLACK)) or (current_pos.pose.position.z > (previous_pos.pose.position.z + self.__SLACK)))):

      print("!!!! position changed !!!!")
      return True
    
    else:
      return False
    

  def eavesdrop_manipulator_moving_state(self):
    return eavesdrop.get_manipulator_moving_state()

  def eavesdrop_current_gripper_state(self):
    return eavesdrop.get_gripper_kinematics_pose()

if __name__ == '__main__':
  print("Commencing MANIPULATOR ENV test...")
  action_space = ManipulatorAction()
  lever = LeverClass()

  env = gym.make('ManipulatorLeverEnv-v0', ManipulatorActions = action_space, LeverInstance = lever, goal_state = 250)

  env.print_gripper_kinematics()

  env.step(3)

  env.print_gripper_kinematics()

  env.step(3)

  env.print_gripper_kinematics()

  print("Queue 15 steps!")
  for i in range(0,15):
    env.step(3)

  print("Testing get_Goal: ")
  print(env.Lever.get_Goal())

  print("Testing get_ABS_pos: ")
  print(env.Lever.get_ABS_Pos())

  print("Testing get_REL_pos: ")
  print(env.Lever.get_REL_Pos())

  print("Testing get_Reward: ")
  print(env.Lever.get_Reward())

  print("Testing env_Ready: ")
  print(env.Lever.env_Ready())

  print("Testing start_Trail: ")
  print(env.Lever.start_Trail())

  print("Testing ABS RESET")
  env.Lever.reset_ABS()
  time.sleep(2)

  print("Testing RAN RESET")
  env.Lever.reset_RAN()
  time.sleep(2)

  env.reset()

  print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
  print("!!!!! Integration test complete !!!!!!!")
  print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")