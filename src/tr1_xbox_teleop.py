#!/usr/bin/env python

import time
import rospy
import sys
import numpy as np
import math
import datetime
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

## right arm
pub_JointRightShoulderPan = rospy.Publisher('/tr1/controller/effort/JointRightShoulderPan/command', Float64, queue_size=10)
pub_JointRightShoulderTilt = rospy.Publisher('/tr1/controller/effort/JointRightShoulderTilt/command', Float64, queue_size=10)
pub_JointRightUpperArmRoll = rospy.Publisher('/tr1/controller/effort/JointRightUpperArmRoll/command', Float64, queue_size=10)
pub_JointRightElbowFlex = rospy.Publisher('/tr1/controller/effort/JointRightElbowFlex/command', Float64, queue_size=10)
pub_JointRightForearmRoll = rospy.Publisher('/tr1/controller/effort/JointRightForearmRoll/command', Float64, queue_size=10)
pub_JointRightWristFlex = rospy.Publisher('/tr1/controller/effort/JointRightWristFlex/command', Float64, queue_size=10)
pub_JointRightWristRoll = rospy.Publisher('/tr1/controller/effort/JointRightWristRoll/command', Float64, queue_size=10)
pub_JointRightGripper = rospy.Publisher('/tr1/controller/effort/JointRightGripper/command', Float64, queue_size=10)

# base
pub_JointBaseWheelFL = rospy.Publisher('/tr1/controller/effort/JointBaseWheelFL/command', Float64, queue_size=10)
pub_JointBaseWheelFR = rospy.Publisher('/tr1/controller/effort/JointBaseWheelFR/command', Float64, queue_size=10)
pub_JointBaseWheelBL = rospy.Publisher('/tr1/controller/effort/JointBaseWheelBL/command', Float64, queue_size=10)
pub_JointBaseWheelBR = rospy.Publisher('/tr1/controller/effort/JointBaseWheelBR/command', Float64, queue_size=10)
pub_JointTorsoExtension = rospy.Publisher('/tr1/controller/effort/JointTorsoExtension/command', Float64, queue_size=10)

# head
pub_neck_base_to_neck = rospy.Publisher('/tr1/controller/effort/JointHeadTilt/command', Float64, queue_size=10)
pub_neck_to_head = rospy.Publisher('/tr1/controller/effort/JointHeadPan/command', Float64, queue_size=10)

# left arm
pub_JointLeftShoulderPan = rospy.Publisher('/tr1/controller/effort/JointLeftShoulderPan/command', Float64, queue_size=10)
pub_JointLeftShoulderTilt = rospy.Publisher('/tr1/controller/effort/JointLeftShoulderTilt/command', Float64, queue_size=10)
pub_JointLeftUpperArmRoll = rospy.Publisher('/tr1/controller/effort/JointLeftUpperArmRoll/command', Float64, queue_size=10)
pub_JointLeftElbowFlex = rospy.Publisher('/tr1/controller/effort/JointLeftElbowFlex/command', Float64, queue_size=10)
pub_JointLeftForearmRoll = rospy.Publisher('/tr1/controller/effort/JointLeftForearmRoll/command', Float64, queue_size=10)
pub_JointLeftWristFlex = rospy.Publisher('/tr1/controller/effort/JointLeftWristFlex/command', Float64, queue_size=10)
pub_JointLeftWristRoll = rospy.Publisher('/tr1/controller/effort/JointLeftWristRoll/command', Float64, queue_size=10)
pub_JointLeftGripper = rospy.Publisher('/tr1/controller/effort/JointLeftGripper/command', Float64, queue_size=10)

mode = 0 # 0 = right arm, 1 = base
increment = 0.05
wristServoValue = 0.5
gripperServoValue = 0
wristServoLeftValue = 0.5
gripperServoLeftValue = 0

old_time = datetime.datetime.now()
def changeMode():
	global mode, old_time;
	current_time = datetime.datetime.now()
	time_diff = current_time - old_time;
	ms_diff = (time_diff.seconds * 1000) + (time_diff.microseconds / 1000);
	if (ms_diff < 100):
		return;
	if (mode == 0):
		mode = 1
		rospy.loginfo("Mode changed to base control")
		old_time = datetime.datetime.now()
		zero_joints()
	elif (mode == 1):
		mode = 2
		rospy.loginfo("Mode changed to left arm control")
		old_time = datetime.datetime.now()
		zero_joints()
	elif (mode == 2):
		mode = 3
		rospy.loginfo("Mode changed to head control")
		old_time = datetime.datetime.now()
		zero_joints()
	elif (mode == 3):
		mode = 0
		rospy.loginfo("Mode changed to right arm control")
		old_time = datetime.datetime.now()
		zero_joints()

def zero_joints():
	pub_JointRightShoulderPan.publish(0)
	pub_JointRightShoulderTilt.publish(0)
	pub_JointRightUpperArmRoll.publish(0)
	pub_JointRightElbowFlex.publish(0)
	pub_JointRightForearmRoll.publish(0)
	pub_JointRightWristFlex.publish(0)
	pub_JointLeftShoulderPan.publish(0)
	pub_JointLeftShoulderTilt.publish(0)
	pub_JointLeftUpperArmRoll.publish(0)
	pub_JointLeftElbowFlex.publish(0)
	pub_JointLeftForearmRoll.publish(0)
	pub_JointLeftWristFlex.publish(0)
	pub_JointBaseWheelFL.publish(0)
	pub_JointBaseWheelFR.publish(0)
	pub_JointBaseWheelBL.publish(0)
	pub_JointBaseWheelBR.publish(0)
	pub_neck_base_to_neck.publish(0)
	pub_neck_to_head.publish(0)

## right arm

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

## left arm

def addWristServoLeftValue():
	global wristServoLeftValue, increment
	if (wristServoLeftValue <= 1 - increment):
		wristServoLeftValue = wristServoLeftValue + increment

def subWristServoLeftValue():
	global wristServoLeftValue, increment
	if (wristServoLeftValue >= 0 + increment):
		wristServoLeftValue = wristServoLeftValue - increment

def addGripperServoLeftValue():
	global gripperServoLeftValue, increment
	if (gripperServoLeftValue <= 1 - increment):
		gripperServoLeftValue = gripperServoLeftValue + increment

def subGripperServoLeftValue():
	global gripperServoLeftValue, increment
	if (gripperServoLeftValue >= 0 + increment):
		gripperServoLeftValue = gripperServoLeftValue - increment

def subscriber_callback(data):
	global mode
	publishData = []

	if (data.buttons[6] == 1):
		changeMode()

	if (mode == 0): # right arm
		pub_JointRightShoulderPan.publish(data.axes[0])
		pub_JointRightShoulderTilt.publish(data.axes[1])
		pub_JointRightUpperArmRoll.publish(data.axes[2])
		pub_JointRightElbowFlex.publish(data.axes[3])
		pub_JointRightForearmRoll.publish(data.axes[6])
		pub_JointRightWristFlex.publish(data.axes[7])

		if (data.buttons[0] == 1):
			addWristServoValue()

		if (data.buttons[1] == 1):
			subWristServoValue()

		if (data.buttons[2] == 1):
			addGripperServoValue()

		if (data.buttons[3] == 1):
			subGripperServoValue()

		pub_JointRightWristRoll.publish(wristServoValue)
		pub_JointRightGripper.publish(gripperServoValue)
	elif (mode == 1): # base
		leftStick = (data.axes[0], data.axes[1])
		rightStickX = data.axes[2]
		motorValues = getMotorValues(np.array(leftStick), rightStickX)
		pub_JointBaseWheelFL.publish(motorValues[0])
		pub_JointBaseWheelFR.publish(motorValues[1] * -1)
		pub_JointBaseWheelBL.publish(motorValues[2])
		pub_JointBaseWheelBR.publish(motorValues[3] * -1)
		pub_JointTorsoExtension.publish(data.axes[7])
	elif (mode == 2): #left arm
		pub_JointLeftShoulderPan.publish(data.axes[0])
		pub_JointLeftShoulderTilt.publish(data.axes[1])
		pub_JointLeftUpperArmRoll.publish(data.axes[2])
		pub_JointLeftElbowFlex.publish(data.axes[3])
		pub_JointLeftForearmRoll.publish(data.axes[6])
		pub_JointLeftWristFlex.publish(data.axes[7])

		if (data.buttons[0] == 1):
			addWristServoLeftValue()

		if (data.buttons[1] == 1):
			subWristServoLeftValue()

		if (data.buttons[2] == 1):
			addGripperServoLeftValue()

		if (data.buttons[3] == 1):
			subGripperServoLeftValue()

		pub_JointLeftWristRoll.publish(wristServoLeftValue)
		pub_JointLeftGripper.publish(gripperServoLeftValue)
	elif (mode == 3):
		pub_neck_base_to_neck.publish(data.axes[0])
		pub_neck_to_head.publish(data.axes[1])

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
	rospy.init_node('tr1_xbox_teleop', anonymous=True)
	rospy.Subscriber("joy", Joy, subscriber_callback)
	rospy.spin()

if __name__ == '__main__':
	do_control()
