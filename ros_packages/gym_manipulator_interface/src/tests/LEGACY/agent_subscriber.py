#!usr/bin/env python

import rospy
from std_msgs.msg import String


def print_received_message():
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def subscribe():
	rospy.init_node("agent_subscriber", anonymous=True)

	rospy.Subscriber("agent_actions", String, print_received_message)

	rospy.spin()


def main():
	print("Here we go!")
	subscribe()

if __name__ == '__main__':
	main()