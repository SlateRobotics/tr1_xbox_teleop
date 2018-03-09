#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

pubs = []
pubs.append(rospy.Publisher('/tr1/arm1_to_arm2_joint_effort_controller/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/arm2_to_arm3_joint_effort_controller/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/arm3_to_arm4_joint_effort_controller/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/arm4_to_arm5_joint_effort_controller/command', Float64, queue_size=10))
#pubs.append(rospy.Publisher('/tr1/arm5_to_arm6_joint_effort_controller/command', Float64, queue_size=10))
#pubs.append(rospy.Publisher('/tr1/arm6_to_arm7_joint_effort_controller/command', Float64, queue_size=10))
#pubs.append(rospy.Publisher('/tr1/arm7_to_arm8_joint_effort_controller/command', Float64, queue_size=10))

def subscriber_callback(data):
	pubs[0].publish(data.axes[0])
	pubs[1].publish(data.axes[1])
	pubs[2].publish(data.axes[4])
	pubs[3].publish(data.axes[3])

def do_control():
	rospy.init_node('tr1_xbox_controller', anonymous=True)
	rospy.Subscriber("joy", Joy, subscriber_callback)
	rospy.spin()

if __name__ == '__main__':
	do_control()
