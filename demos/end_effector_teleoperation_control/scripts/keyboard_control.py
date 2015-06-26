#!/usr/bin/env python

import argparse

import rospy

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION

from std_msgs.msg import (
	String,
)

global_command = "keep"
def map_keyboard():
	pub = rospy.Publisher("end_effector_command", String, queue_size=10);
	keyboard_binding = {}
	keyboard_binding["w"] = "up"
	keyboard_binding["s"] = "down"
	keyboard_binding["a"] = "left"
	keyboard_binding["d"] = "right"
	keyboard_binding["q"] = "backward"
	keyboard_binding["e"] = "forward"
	keyboard_binding["f"] = "switch limb to "
	keyboard_binding["z"] = "orientation_x"
	keyboard_binding[" "] = "keep"
	keyboard_binding["k"] = "further"
	keyboard_binding["l"] = "closer"
	limb = ["right", "left"]
	current_limb = 0
	print limb[current_limb] + " limb under control..."
	rate = rospy.Rate(6) #6Hz
	while not rospy.is_shutdown():
		c = baxter_external_devices.getch()
		if c:
			#catch Esc or ctrl-c
			if c in ['\x1b', '\x03']:
				rospy.signal_shutdown("Finished.Exiting...")
			if(c in keyboard_binding.keys()):
				if (c == "f"):
					current_limb = 1 - current_limb
					print "control switch to " + limb[current_limb]
					pub.publish(String(keyboard_binding[c] + limb[current_limb]))
				else:
					print "sending command: " + limb[current_limb] + " limb " + keyboard_binding[c]
					pub.publish(String(keyboard_binding[c]))
			else:
				print "invalid command: " + c
			#print limb[current_limb] + " limb under control..."
			rate.sleep()


def main():
	print("Initializing node keyboard_control... ")
	rospy.init_node("keyboard_control")

	try:
		map_keyboard()
	except():
		pass
	print("Done.")

if __name__ == '__main__':
	main()
