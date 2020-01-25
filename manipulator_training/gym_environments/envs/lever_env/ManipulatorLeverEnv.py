
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

  def __init__(self, 
                ManipulatorActions, 
                LeverInstance, 
                lever_goal_position = 0, 
                use_platform = False, 
                manipulator_random_init = False, 
                lever_random_init = False
              ):

    self.ManipulatorActions = ManipulatorActions
    self.Lever = LeverInstance
    self.Lever.set_Goal(lever_goal_position)


    self._SLACK = 0.001
    self._JOINT_BOUNDS = [[-3.00, 3.00],
                          [-1.21, -0.41],
                          [-1.35, 0.85],
                          [-1.6, -0.14]]
    
    eavesdrop.init_eavesdropper()


    self.previous_gripper_pose    = self._get_current_gripper_pose()
    self.current_gripper_pose     = self._get_current_gripper_pose()
    self.previous_lever_rel_pos   = self.Lever.get_REL_Pos()
    self.current_lever_rel_pos    = self.Lever.get_REL_Pos()

    #TASK PARAMS
    self.action_space             = spaces.Discrete(len(self.ManipulatorActions.getActionSpace()))
    self.use_platform             = use_platform
    self.reset_manipulator_RANDOM = manipulator_random_init
    self.reset_lever_RANDOM       = lever_random_init
    self.state                    = None

    
    
  def step(self, action):
    #PERFORMING MANIPULATOR ACTION
    if(self.ManipulatorActions.validAction(action)):
      self.ManipulatorActions.queueAction(action)
      self._wait_for_action_execution()
    else:
      print("---------------------------------")
      print("-------- INVALID ACTION ---------")
      print("---------------------------------")

    #CHECKING LEVER STATE
    #self.previous_lever_rel_pos = self.current_lever_rel_pos
    #self.current_lever_rel_pos = self.Lever.get_REL_Pos()

    self._update_state(self._get_current_joint_state(), 
                        self._get_current_gripper_pose(), 
                        self.Lever.get_REL_Pos()
                        )

    #OUT OF BOUNDS CHECK
    if(self._out_of_bounds()):
      terminal = True
      reward = -500

    else:
      terminal = self._terminal()
      
      if(terminal):
        reward = 0
      else:
        reward = -1

    return (self._get_obs(), reward, terminal, {}) 
      


  def reset(self):

    print("---------------------------------------")
    print("-------- RESETTING ENVIRONMENT --------")
    print("---------------------------------------")

    #RESETTING MANIPULATOR
    if(self.reset_manipulator_RANDOM):
      self.ManipulatorActions.reset_RANDOM()
    else:
      self.ManipulatorActions.reset_IDEAL()

    self._wait_for_action_execution()

    #RESETTING LEVER
    if(self.reset_lever_RANDOM):
      self.Lever.reset_RAN()
    else:
      self.Lever.reset_MID()

    #This is ment to be blocking, so that the environment has time to reset before new querries. 
    lever_ready = False
    manipulator_moving = True
    while(not(lever_ready and (not manipulator_moving))):
      lever_ready = self.Lever.env_Ready()
      manipulator_moving = self._get_manipulator_moving_state()
      print("Lever ready: {}".format(lever_ready))
      print("Manipulator moving: {}".format(manipulator_moving))
      print("... waiting for reset ...")
      time.sleep(1)

    self.state = None
    
    self.previous_gripper_pose = self._get_current_gripper_pose()
    self.previous_lever_rel_pos = self.Lever.get_REL_Pos()

    self._update_state(self._get_current_joint_state(),
                        self._get_current_gripper_pose(), 
                        self.Lever.get_REL_Pos())

    return self._get_obs()


  def render(self, mode='human', close = False):
    print("No rendering. Its a physical system.")

  def _update_state(self, manipulator_joint_states, manipulator_gripper_pose, leverState):
    self.previous_gripper_pose = self.current_gripper_pose
    self.current_gripper_pose = manipulator_gripper_pose
    #leverState is the levers relative distance to its goal
    self.previous_lever_rel_pos = self.current_lever_rel_pos
    self.current_lever_rel_pos = leverState

    self.state = [manipulator_joint_states, manipulator_gripper_pose, leverState]

  def _get_obs(self):
      tmp = self.state
      return [tmp[0].position, tmp[1].pose, tmp[2]]
  
  def _terminal(self):
      if(self.Lever.get_Terminal()):
        return True
      else:
        return False

  def _out_of_bounds(self):
    joint_values = self.state[0].position
    for i in range(len(self._JOINT_BOUNDS)):
       if((joint_values[i] < self._JOINT_BOUNDS[i][0]) or (self._JOINT_BOUNDS[i][1] < joint_values[i])):
        return True
    return False

  def print_gripper_kinematics(self):
      print("---------------------------------------")
      print("PREVIOUS GRIPPER STATE: ")
      print(self.previous_gripper_pose)
      print("---------------------------------------")
      print("CURRENT GRIPPER STATE: ")
      print(self.current_gripper_pose)
      print("---------------------------------------")

  def _wait_for_action_execution(self):
    #BLOCKING!
    if(self.use_platform):
      while self._get_manipulator_moving_state():
        time.sleep(0.1)
        continue
    elif(not self.use_platform):
      while(not self._gripper_position_changed(self.previous_gripper_pose, self.current_gripper_pose)):
        self.current_gripper_pose = self._get_current_gripper_pose()
        time.sleep(0.1)

  def _gripper_position_changed(self, previous_pos, current_pos):
    if(((current_pos.pose.position.x <= (previous_pos.pose.position.x - self._SLACK)) or (current_pos.pose.position.x > (previous_pos.pose.position.x + self._SLACK))) 
        or ((current_pos.pose.position.y <= (previous_pos.pose.position.y - self._SLACK)) or (current_pos.pose.position.y > (previous_pos.pose.position.y + self._SLACK)))
        or ((current_pos.pose.position.z <= (previous_pos.pose.position.z - self._SLACK)) or (current_pos.pose.position.z > (previous_pos.pose.position.z + self._SLACK)))):
      return True
    
    else:
      return False
    

  def _get_manipulator_moving_state(self):
    return eavesdrop.get_manipulator_moving_state()

  def _get_current_gripper_pose(self):
    return eavesdrop.get_manipulator_gripper_kinematics_pose()

  def _get_current_joint_state(self):
    return eavesdrop.get_manipulator_joint_states()

if __name__ == '__main__':
  print("Commencing MANIPULATOR ENV test...")
  action_space = ManipulatorAction()
  lever = LeverClass()

  env = gym.make('ManipulatorLeverEnv-v0', ManipulatorActions = action_space, LeverInstance = lever, lever_goal_position = 250)

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

  print("Testing get_Terminal: ")
  print(env.Lever.get_Terminal())

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