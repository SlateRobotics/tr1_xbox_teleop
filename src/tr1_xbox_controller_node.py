#!/usr/bin/env python

import time
import rospy
import sys
import numpy as np
import math
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
pubs.append(rospy.Publisher('/tr1/controller/effort/JointBaseWheelFL/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/controller/effort/JointBaseWheelFR/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/controller/effort/JointBaseWheelBL/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/controller/effort/JointBaseWheelBR/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/controller/effort/JointTorsoExtension/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/controller/effort/neck_base_to_neck/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/controller/effort/neck_to_head/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/controller/effort/JointLeftShoulderPan/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/controller/effort/JointLeftShoulderTilt/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/controller/effort/JointLeftUpperArmRoll/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/controller/effort/JointLeftElbowFlex/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/controller/effort/JointLeftForearmRoll/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/controller/effort/JointLeftWristFlex/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/controller/effort/JointLeftWristRoll/command', Float64, queue_size=10))
pubs.append(rospy.Publisher('/tr1/controller/effort/JointLeftGripper/command', Float64, queue_size=10))

mode = 0 # 0 = right arm, 1 = base
increment = 0.05
wristServoValue = 0.5
gripperServoValue = 0

def changeMode():
	global mode
	if (mode == 0):
		mode = 1
		rospy.loginfo("Mode changed to base control")
	elif (mode == 1):
		mode = 2
		rospy.loginfo("Mode changed to left arm control")
	elif (mode == 2):
		mode = 3
		rospy.loginfo("Mode changed to head control")
	elif (mode == 3):
		mode = 0
		rospy.loginfo("Mode changed to right arm control")

def addWristServoValue():
	global wristServoValue, increment
	if (wristServoValue <= 1 - increment):
		wristServoValue = wristServoValue + increment

def subWristServoValue():
	global wristServoValue, increment
	if (wristServoValue >= 0 + increment):
		wristServoValue = wristServoValue - increment

def addGripperServoValue():
	global gripperServoValue, increment
	if (gripperServoValue <= 1 - increment):
		gripperServoValue = gripperServoValue + increment

def subGripperServoValue():
	global gripperServoValue, increment
	if (gripperServoValue >= 0 + increment):
		gripperServoValue = gripperServoValue - increment

def subscriber_callback(data):
	global mode
	publishData = []

	if (data.buttons[6] == 1):
		changeMode()

	if (mode == 0):
		publishData.append(data.axes[0])
		publishData.append(data.axes[1])
		publishData.append(data.axes[2])
		publishData.append(data.axes[3])
		publishData.append(data.axes[6])
		publishData.append(data.axes[7])

		if (data.buttons[0] == 1):
			addWristServoValue()

		if (data.buttons[1] == 1):
			subWristServoValue()

		if (data.buttons[2] == 1):
			addGripperServoValue()

		if (data.buttons[3] == 1):
			subGripperServoValue()

		publishData.append(wristServoValue)
		publishData.append(gripperServoValue)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
	elif (mode == 1):
		publishData.append(0)   
		publishData.append(0)
		publishData.append(0) 
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(wristServoValue)
		publishData.append(gripperServoValue)
		leftStick = (data.axes[0], data.axes[1])
		rightStickX = data.axes[2]
		motorValues = getMotorValues(np.array(leftStick), rightStickX)
		publishData.append(motorValues[0])
		publishData.append(motorValues[1] * -1)
		publishData.append(motorValues[2])
		publishData.append(motorValues[3] * -1)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
	elif (mode == 2):
		publishData.append(0)   
		publishData.append(0)
		publishData.append(0) 
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(data.axes[0])
		publishData.append(data.axes[1])
		publishData.append(data.axes[2])
		publishData.append(data.axes[3])
		publishData.append(data.axes[6])
		publishData.append(data.axes[7])

		if (data.buttons[0] == 1):
			addWristServoValue()

		if (data.buttons[1] == 1):
			subWristServoValue()

		if (data.buttons[2] == 1):
			addGripperServoValue()

		if (data.buttons[3] == 1):
			subGripperServoValue()

		publishData.append(wristServoValue)
		publishData.append(gripperServoValue)
	elif (mode == 3):
		publishData.append(0)   
		publishData.append(0)
		publishData.append(0) 
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(data.axes[0])
		publishData.append(data.axes[1])
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)
		publishData.append(0)

	# rospy.loginfo("Joystick: %f, %f, %f, %f, %f, %f, %f, %f", publishData[0], publishData[1], publishData[2], publishData[3], publishData[4], publishData[5], publishData[6], publishData[7])

	# need to publish data on pubs: pubs[i].publish(publishdata[i])
	for idx, val in enumerate(publishData):
		pubs[idx].publish(publishData[idx])

# input: numpy array, -1 to 1 value
def getMotorValues(desiredVector, rotationStrength):
	if desiredVector.size == 0 and rotationStrength == 0:
		return (0, 0, 0, 0)

	desiredX = desiredVector.tolist()[0]
	desiredY = desiredVector.tolist()[1]
	desiredMagnitude = abs(desiredX)
	if abs(desiredY) > desiredMagnitude:
		desiredMagnitude = abs(desiredY)
	if abs(rotationStrength) > desiredMagnitude:
		desiredMagnitude = abs(rotationStrength)

	if desiredMagnitude == 0:
		return (0, 0, 0, 0)

	# create rotation matrix
	offset = math.pi / 4
	c, s = np.cos(offset), np.sin(offset)
	rotationMatrix = np.matrix('{} {}; {} {}'.format(c, -s, s, c))

	# create output vector from rotation matrix and desiredVector
	ouput = desiredVector.dot(rotationMatrix).tolist()[0]
	outputX = ouput[0]
	outputY = ouput[1]

	# define motors
	frontLeft = outputX
	frontRight = outputY
	backLeft = outputY
	backRight = outputX

	#manipulate values based on rotation from rotationStrength
	rotationStrength = rotationStrength * 2
	frontLeft = frontLeft + rotationStrength
	frontRight = frontRight - rotationStrength
	backLeft = backLeft + rotationStrength
	backRight = backRight - rotationStrength

	# get output magnitude
	outputMagnitude = abs(frontRight)
	if abs(frontLeft) > outputMagnitude:
		outputMagnitude = abs(frontLeft)
	if abs(backLeft) > outputMagnitude:
		outputMagnitude = abs(backLeft)
	if abs(backRight) > outputMagnitude:
		outputMagnitude = abs(backRight)

	# scale output to match desired magnitude
	scale = desiredMagnitude / outputMagnitude

	frontLeft = frontLeft * scale
	frontRight = frontRight * scale
	backLeft = backLeft * scale
	backRight = backRight * scale

	scaledOutput = (frontRight, frontLeft, backLeft, backRight)

	return scaledOutput


def do_control():
	rospy.init_node('tr1_xbox_controller', anonymous=True)
	rospy.Subscriber("joy", Joy, subscriber_callback)
	rospy.spin()

if __name__ == '__main__':
	do_control()
