#!/usr/bin/env python

import argparse
import copy

import rospy
import numpy

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics

from baxter_core_msgs.msg import (
	JointCommand
)
from std_msgs.msg import (
	String,
)

import collections
Point = collections.namedtuple('Point', ['x', 'y', 'z'])
Quaternion = collections.namedtuple('Quaternion', ['x', 'y', 'z', 'w'])

pub = rospy.Publisher("end_effector_command_solution", JointCommand, queue_size=1)

current_limb = "right"
global_distance = 0.1
def callback(data):
	global current_limb
	global global_distance
	command = data.data
	if not (command.find("switch") == -1):
		if (command.find("right") == -1):
			current_limb = "left"
		else:
			current_limb = "right"
		return
	limb = baxter_interface.Limb(current_limb)
	limbPose = limb.endpoint_pose()
	while not ("position" in limbPose):
		#print "re-getting pose"
		limbPose = limb.endpoint_pose()
	x = 0
	y = 0
	z = 0
	rx = 0
	ry = 0
	rz = 0
	rw = 0
	if (command == "up"):
		z = global_distance
	elif (command == "down"):
		z = -global_distance
	elif (command == "left"):
		y = global_distance
	elif (command == "right"):
		y = -global_distance
	elif (command == "backward"):
		x = -global_distance
	elif (command == "forward"):
		x = global_distance
	elif (command == "orientation_x"):
		rx = global_distance
	elif (command == "keep"):
		pass
	elif (command == "further"):
		if (global_distance < 0.3):
			global_distance += 0.01
			print global_distance
		else:
			print "can not increase more"
	elif (command == "closer"):
		if (global_distance > 0):
			global_distance -= 0.01
			if (global_distance < 0):
				global_distance = 0
			print global_distance
		else:
			print "can not decrease more"
	else:
		print "unknown command"
		return
	newPose = dict()
	newPose = {
		'position': Point(
				limbPose["position"].x + x,
				limbPose["position"].y + y,
				limbPose["position"].z + z,
			),
		'orientation': Quaternion(
				limbPose["orientation"].x + rx,
				limbPose["orientation"].y,
				limbPose["orientation"].z,
				limbPose["orientation"].w,
			),
	}
	kinematics = baxter_kinematics(current_limb)
	inverseKinematics = kinematics.inverse_kinematics(newPose["position"], newPose["orientation"])
	
	if not (inverseKinematics == None):
		inverseKinematicsSolution = list()
		for num in inverseKinematics.tolist():
			inverseKinematicsSolution.append(num)
		inverseKinematicsSolutionJointCommand = JointCommand()
		inverseKinematicsSolutionJointCommand.mode = 1
		inverseKinematicsSolutionJointCommand.names = limb.joint_names()
		inverseKinematicsSolutionJointCommand.command = inverseKinematicsSolution
		pub.publish(inverseKinematicsSolutionJointCommand)

def subscribe():
	rospy.Subscriber("/end_effector_command", String, callback)
	print "subscribing.."
	rospy.spin();
def main():
	print("Initializing node control_command_subscriber... ")
	rospy.init_node("control_command_subscriber", anonymous=True)

	try:
		subscribe()
	except():
		pass
	print("Done.")

if __name__ == '__main__':
	main()
