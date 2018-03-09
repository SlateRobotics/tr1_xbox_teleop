#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

pubs = []
pubs.append(rospy.Publisher('/tr1/arm1_to_arm2_joint_effort_controller/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/arm2_to_arm3_joint_effort_controller/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/arm3_to_arm4_joint_effort_controller/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/arm4_to_arm5_joint_effort_controller/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/arm5_to_arm6_joint_effort_controller/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/arm6_to_arm7_joint_effort_controller/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/arm7_to_arm8_joint_effort_controller/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/gripper_joint_effort_controller/command', Float64, queue_size=10))

increment = 0.1
wristServoValue = 0
gripperServoValue = 1

def addWristServoValue():
	global wristServoValue, increment
	if (wristServoValue <= 1 - increment):
		wristServoValue = wristServoValue + increment

def subWristServoValue():
	global wristServoValue, increment
	if (wristServoValue >= -1 + increment):
		wristServoValue = wristServoValue - increment

def addGripperServoValue():
	global gripperServoValue, increment
	if (gripperServoValue <= 1 - increment):
		gripperServoValue = gripperServoValue + increment

def subGripperServoValue():
	global gripperServoValue, increment
	if (gripperServoValue >= -1 + increment):
		gripperServoValue = gripperServoValue - increment

def subscriber_callback(data):
	if (data.buttons[0] == 1):
		pubs[0].publish(0)
		pubs[1].publish(0)
		pubs[2].publish(0)
		pubs[3].publish(0)
		pubs[4].publish(0)
		pubs[5].publish(0)
		pubs[6].publish(data.axes[0])
		pubs[7].publish(data.axes[1])

	else:
		pubs[0].publish(data.axes[0])
		pubs[1].publish(data.axes[1])
		pubs[2].publish(data.axes[4])
		pubs[3].publish(data.axes[3])

		if (data.buttons[11] == 1):
			pubs[4].publish(1)
		elif (data.buttons[12] == 1):
			pubs[4].publish(-1)
		else:
			pubs[4].publish(0)

		if (data.buttons[13] == 1):
			pubs[5].publish(1)
		elif (data.buttons[14] == 1):
			pubs[5].publish(-1)
		else:
			pubs[5].publish(0)

		pubs[6].publish(wristServoValue)
		pubs[7].publish(gripperServoValue)

def do_control():
	rospy.init_node('tr1_xbox_controller', anonymous=True)
	rospy.Subscriber("joy", Joy, subscriber_callback)
	rospy.spin()

if __name__ == '__main__':
	do_control()
