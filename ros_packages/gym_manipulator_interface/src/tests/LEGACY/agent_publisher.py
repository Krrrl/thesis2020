#!usr/bin/env python

import rospy

from std_msgs.msg import String

import random as rn

action_space = {0:'y', 
								1:'h',
								2:'u',
								3:'j',
								4:'i',
								5:'k',
								6:'o',
								7:'l',
								8:'g',
								9:'f'}


action_queue = []


def update_action_queue():
	#just adds a random action
	action_queue.append(rn.sample(action_space,1)[0])

def action_publisher():

	pub = rospy.Publisher("agent_actions", String, queue_size = 10)
	rospy.init_node("agent_actions", anonymous=True)
	rate = rospy.Rate(100)

	while not rospy.is_shutdown():
		update_action_queue()
		if(action_queue):
			action = action_queue.pop()

		else:
			action = "0"

		rospy.loginfo(action)
		pub.publish(action)
		rate.sleep()

if __name__ == '__main__':
	try:
		action_publisher()
	except rospy.ROSInterruptException:
		pass
