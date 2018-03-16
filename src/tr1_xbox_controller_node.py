#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

pubs = []
pubs.append(rospy.Publisher('/tr1/controller/effort/JointRightShoulderPan/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/controller/effort/JointRightShoulderTilt/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/controller/effort/JointRightUpperArmRoll/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/controller/effort/JointRightElbowFlex/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/controller/effort/JointRightForearmRoll/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/controller/effort/JointRightWristFlex/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/controller/effort/JointRightWristRoll/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/controller/effort/JointRightGripper/command', Float64, queue_size=10))

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
	publishData = []

	if (data.buttons[0] == 1):
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(data.axes[0])
		publishData.append(data.axes[1])

	else:
		publishData.append(data.axes[0])
		publishData.append(data.axes[1])
		publishData.append(data.axes[3])
		publishData.append(data.axes[4])

		if (data.buttons[11] == 1):
			publishData.append(1)
		elif (data.buttons[12] == 1):
			publishData.append(-1)
		else:
			publishData.append(0)

		if (data.buttons[13] == 1):
			publishData.append(1)
		elif (data.buttons[14] == 1):
			publishData.append(-1)
		else:
			publishData.append(0)

		publishData.append(wristServoValue)
		publishData.append(gripperServoValue)

	# rospy.loginfo("Joystick: %f, %f, %f, %f, %f, %f, %f, %f", publishData[0], publishData[1], publishData[2], publishData[3], publishData[4], publishData[5], publishData[6], publishData[7])

	# need to publish data on pubs: pubs[i].publish(publishdata[i])
	for idx, val in enumerate(publishData):
		pubs[idx].publish(publishData[idx])

def do_control():
	rospy.init_node('tr1_xbox_controller', anonymous=True)
	rospy.Subscriber("joy", Joy, subscriber_callback)
	rospy.spin()

if __name__ == '__main__':
	do_control()
