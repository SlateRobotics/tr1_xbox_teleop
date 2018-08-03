#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from math import pi
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('tr1_xbox_moveit_teleop_node', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("right_arm")

def subscriber_callback(data):
	global group
	pose_goal = group.get_current_pose().pose
	pose_goal.position.x += data.axes[1] * 0.01
	pose_goal.position.y += data.axes[0] * 0.01
	pose_goal.position.z += data.axes[3] * 0.01
	print pose_goal.position.x, pose_goal.position.y, pose_goal.position.z
	group.set_pose_target(pose_goal)

	plan = group.go(wait=True)
	group.stop()
	group.clear_pose_targets()

rospy.Subscriber("joy", Joy, subscriber_callback)
rospy.spin()

