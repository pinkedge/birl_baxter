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
    PoseStamped,
)
import collections
Point = collections.namedtuple('Point', ['x', 'y', 'z'])
Quaternion = collections.namedtuple('Quaternion', ['x', 'y', 'z', 'w'])

pub = rospy.Publisher("end_effector_command_solution", JointCommand, queue_size=1)

def callback(data):
	time_now = rospy.get_time()
	if (time_now - data.header.stamp.secs > 1):
		print "time stamp too old, ignoring..."
		return
	print "accept command"
	commandPose = data.pose
	limb_name = data.header.frame_id;
	limb = baxter_interface.Limb(limb_name)
	currentPose = limb.endpoint_pose()
	while not ("position" in currentPose):
		currentPose = limb.endpoint_pose()
	newPose = dict()
	newPose = {
		'position': Point(
				commandPose.position.x,
				commandPose.position.y,
				commandPose.position.z,
			),
		'orientation': Quaternion(
				commandPose.orientation.x,
				commandPose.orientation.y,
				commandPose.orientation.z,
				commandPose.orientation.w,
			),
	}
	kinematics = baxter_kinematics(limb_name)
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
	rospy.Subscriber("end_effector_command_position", PoseStamped, callback)
	print "end_effector_command_position subscribing.."
	rospy.spin();


def main():
	print("Initializing node end_effector_command_solver... ")
	rospy.init_node("end_effector_command_solver", anonymous=True)

	try:
		subscribe()
	except():
		pass
	print("Done.")

if __name__ == '__main__':
	main()
