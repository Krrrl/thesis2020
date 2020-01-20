#! /usr/bin/env python


import gripper_position_listener

import time

if __name__ == '__main__':
	print("Awake!")
	listener = gripper_position_listener.gripper_state_listener(10)
	print("Created listener!")
	listener.run()
	print("Running!")

	while True:
		print("Going to sleep")
		time.sleep(1)
		data = gripper_position_listener.get_gripper_position()
		print("new data: {}".format(data))
		print("HERE IS JUST THE X VALUE! {}".format(data.pose.position.x))