#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy

def callback(data):
	rospy.loginfo(data)

def do_control():
	rospy.init_node('tr1_xbox_controller', anonymous=True)
	rospy.Subscriber("joy", Joy, callback)
	rospy.spin()

if __name__ == '__main__':
	do_control()
