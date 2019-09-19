#!/usr/bin/env python3

import time
import rospy
from libcozmo.msg import ActionMsg
import cozmo
from cozmo.util import radians

class CozmoActionNode(object):
	""" ROS node to execute actions on Cozmo

	This node implements ROS to receive a sequence of actions from the solver,
	and processes them to the Cozmo SDK for the robot to execute.

	Attributes:
		subscriber: ROS subscriber node
		heading: Heading of action (radians)
		duraton: Duration of action (seconds)
		speed: Speed of action (millimiters/second)
	"""
	def __init__(self):
		"""Constructs subscriber for receiving action messages"""
		self.subscriber = rospy.Subscriber( \
			"Action", ActionMsg, self.callback, queue_size = 10)
		rospy.spin()

	def callback(self, data):
		"""
		Callback function for listener node.

		@param data: Data from ROS message
		"""
		self.heading = data.heading
		self.duration = data.duration
		self.speed = data.speed
		print("I heard the following action: heading: %f duration: %f \
			speed: %f" % (self.heading, self.duration, self.speed))
		cozmo.run_program(self.move_cozmo)

	def move_cozmo(self, robot= cozmo.robot.Robot):
		"""
		Move cozmo with given speed, heading, and duration.

		@param robot: cozmo to execute action on
		"""
		robot.turn_in_place(
			radians(self.heading), is_absolute = True).wait_for_completed()
		robot.drive_wheels( \
			self.speed, self.speed, 500, 500, duration = self.duration)

if __name__ == '__main__':
	rospy.init_node("cozmo", anonymous = True)
	print("node intialized, waiting for messages..")
	try:
		node = CozmoActionNode()
	except rospy.ROSInterruptException:
		pass

