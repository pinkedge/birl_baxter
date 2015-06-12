#!/usr/bin/env python

import argparse

import rospy

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION

from std_msgs.msg import (
	String,
)


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
	limb = ["right", "left"]
	current_limb = 0
	print limb[current_limb] + " limb under control..."
	while not rospy.is_shutdown():
		c = baxter_external_devices.getch()
		if c:
			#catch Esc or ctrl-c
			if c in ['\x1b', '\x03']:
				rospy.signal_shutdown("Finished.Exiting...")
			elif(c in keyboard_binding.keys()):
				if (c == "f"):
					current_limb = 1 - current_limb
					print "control switch to " + limb[current_limb]
					pub.publish(String(keyboard_binding[c] + limb[current_limb]))
				else:
					print "sending command: " + limb[current_limb] + " limb move " + keyboard_binding[c]
					pub.publish(String(keyboard_binding[c]))
			else:
				print "invalid command: " + c
			#print limb[current_limb] + " limb under control..."


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
