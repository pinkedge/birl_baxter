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
	Header,
)
from geometry_msgs.msg import (
    Point,
    Quaternion,
    Pose,
    PoseStamped,
)

pub = rospy.Publisher("end_effector_command_position", PoseStamped, queue_size=1)

current_limb = "right"
global_distance = 0.01
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
			global_distance += 0.05
			print global_distance
		else:
			print "can not increase more"
	elif (command == "closer"):
		if (global_distance > 0):
			global_distance -= 0.05
			if (global_distance < 0):
				global_distance = 0
			print global_distance
		else:
			print "can not decrease more"
	else:
		print "unknown command"
		return
	newPose = Pose()
	newPose.position = Point(
				limbPose["position"].x + x,
				limbPose["position"].y + y,
				limbPose["position"].z + z,
			)
	newPose.orientation = Quaternion(
				limbPose["orientation"].x + rx,
				limbPose["orientation"].y,
				limbPose["orientation"].z,
				limbPose["orientation"].w,
			)

	newHeader = Header()
	newHeader.frame_id = current_limb

	newHeader.stamp = rospy.Time().now()

	newPoseStamp = PoseStamped()
	newPoseStamp.pose = newPose
	newPoseStamp.header = newHeader

	pub.publish(newPoseStamp)

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
