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
	while not rospy.is_shutdown():
		c = baxter_external_devices.getch()
		if c:
			#catch Esc or ctrl-c
			if c in ['\x1b', '\x03']:
				rospy.signal_shutdown("Finished.Exiting...")
			elif(c in keyboard_binding.keys()):
				print "sending command: " + keyboard_binding[c]
				pub.publish(String(keyboard_binding[c]))
			else:
				print "invalid command"


def main():
	print("Initializing node... ")
	rospy.init_node("keyboard_control")

	def clean_shutdown():
		print("\nExiting...")
	rospy.on_shutdown(clean_shutdown)

	try:
		map_keyboard()
	except():
		pass
	print("Done.")

if __name__ == '__main__':
	main()
