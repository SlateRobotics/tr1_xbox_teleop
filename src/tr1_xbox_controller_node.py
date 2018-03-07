#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

pub = rospy.Publisher('/tr1/arm1_to_arm2_joint_effort_controller/command', Float64, queue_size=10)

def subscriber_callback(data):
	joint1_effort = data.axes[0]
	rospy.loginfo(joint1_effort)
	pub.publish(joint1_effort)

def do_control():
	rospy.init_node('tr1_xbox_controller', anonymous=True)
	rospy.Subscriber("joy", Joy, subscriber_callback)
	rospy.spin()

if __name__ == '__main__':
	do_control()
