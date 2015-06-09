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

pub = rospy.Publisher("end_effector_command_solution", JointCommand, queue_size=10)

def callback(data):
	command = data.data
	print "I recieved command:" + command
	rightLimb = baxter_interface.Limb('right')
	limbPose = rightLimb.endpoint_pose()
	while not ("position" in limbPose):
		#print "re-getting pose"
		limbPose = rightLimb.endpoint_pose()

	x = 0
	y = 0
	z = 0
	if (command == "up"):
		z = 0.1
	elif (command == "down"):
		z -= 0.1
	elif (command == "left"):
		x += 0.1
	elif (command == "right"):
		x -= 0.1
	elif (command == "backward"):
		y -= 0.1
	elif (command == "forward"):
		y += 0.1
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
				limbPose["orientation"].x,
				limbPose["orientation"].y,
				limbPose["orientation"].z,
				limbPose["orientation"].w,
			),
	}
	print "after command:"
	print newPose
	rightKinematics = baxter_kinematics('right')
	rightInverseKinematics = rightKinematics.inverse_kinematics(newPose["position"], newPose["orientation"])
	print "ik solution:"
	print rightInverseKinematics
	
	if not (rightInverseKinematics == "None"):
		rightInverseKinematicsSolution = list()
		for num in rightInverseKinematics.tolist():
			rightInverseKinematicsSolution.append(num)
		rightInverseKinematicsSolutionJointCommand = JointCommand()
		rightInverseKinematicsSolutionJointCommand.mode = 1
		rightInverseKinematicsSolutionJointCommand.names = rightLimb.joint_names()
		rightInverseKinematicsSolutionJointCommand.command = rightInverseKinematicsSolution
		pub.publish(rightInverseKinematicsSolutionJointCommand)

def subscribe():
	rospy.Subscriber("/end_effector_command", String, callback)
	print "subscribing.."
	rospy.spin();


def main():
	print("Initializing node... ")
	rospy.init_node("control_command_subscriber", anonymous=True)

	def clean_shutdown():
		print("\nExiting...")
	rospy.on_shutdown(clean_shutdown)

	try:
		subscribe()
	except():
		pass
	print("Done.")

if __name__ == '__main__':
	main()
