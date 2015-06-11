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
from geometry_msgs.msg import (
    Pose,
)
import collections
Point = collections.namedtuple('Point', ['x', 'y', 'z'])
Quaternion = collections.namedtuple('Quaternion', ['x', 'y', 'z', 'w'])

pub = rospy.Publisher("end_effector_command_solution", JointCommand, queue_size=1)

def callback(data):
	print data
	limb = baxter_interface.Limb("right")
	currentPose = limb.endpoint_pose()
	newPose = dict()
	newPose = {
		'position': Point(
				data.position.x,
				data.position.y,
				data.position.z,
			),
		'orientation': Quaternion(
				currentPose["orientation"].x,
				currentPose["orientation"].y,
				currentPose["orientation"].z,
				currentPose["orientation"].w,
			),
	}
	kinematics = baxter_kinematics("right")
	inverseKinematics = kinematics.inverse_kinematics(newPose["position"], newPose["orientation"])
	print inverseKinematics
	
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
	rospy.Subscriber("end_effector_command_position", Pose, callback)
	print "subscribing.."
	rospy.spin();


def main():
	print("Initializing node... ")
	rospy.init_node("end_effector_command_solver", anonymous=True)

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
