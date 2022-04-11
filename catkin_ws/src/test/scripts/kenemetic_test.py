#!/usr/bin/env python3.8

from socket import MSG_NOSIGNAL
import os
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose, Point, Quaternion
import math as m

class ur10e():
	def __init__(self, nodeName, groupName):
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node(nodeName, anonymous=True)
		self.pose = Pose()
		self.moveGroup = moveit_commander.MoveGroupCommander(groupName)
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.display_trajectory_pub = rospy.Publisher("move_group/display_planned_path", DisplayTrajectory, queue_size=20)
		self.running = True

	def setPos(self, x, y, z):
		self.pose.orientation.w, self.pose.position.x, self.pose.position.y, self.pose.position.z = self.__calcQuaternions(x, y, z)


	def moveToPos(self):
		print("W orientation: ", self.pose.orientation.w, ", x value: ", self.pose.position.x, ", y value: ", self.pose.position.y, ", z value: ", self.pose.position.z)
		self.moveGroup.set_pose_target(self.pose)
		self.moveGroup.go(wait=True)
		self.moveGroup.stop()
		self.moveGroup.clear_pose_targets()

	def __calcQuaternions(self, phi, theta, psi):
		qw = m.cos(phi/2) * m.cos(theta/2) * m.cos(psi/2) + m.sin(phi/2) * m.sin(theta/2) * m.sin(psi/2)
		qx = m.sin(phi/2) * m.cos(theta/2) * m.cos(psi/2) - m.cos(phi/2) * m.sin(theta/2) * m.sin(psi/2)
		qy = m.cos(phi/2) * m.sin(theta/2) * m.cos(psi/2) + m.sin(phi/2) * m.cos(theta/2) * m.sin(psi/2)
		qz = m.cos(phi/2) * m.cos(theta/2) * m.sin(psi/2) - m.sin(phi/2) * m.sin(theta/2) * m.cos(psi/2)
		return qw, qx, qy, qz

	def cleanUp(self):
		if(self.running):
			rospy.spin()
			moveit_commander.roscpp_shutdown()
			self.running = False

	def __del__(self):
		self.cleanUp()

ur = ur10e('ur10_e_move_test', "manipulator")

while (True):
	x = float(input("give X value:\n"))
	y = float(input("give Y value:\n"))
	z = float(input("give Z value:\n"))
	ur.setPos(1,2,3)
	ur.moveToPos()
	str = input ("give 'y' to keep sending commands, 'n' to stop sendig\n")
	if (str == "n"):
		break

ur.cleanUp()

