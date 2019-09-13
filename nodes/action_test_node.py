#!/usr/bin/env python3

"""
This file implementes ROS subscriber and Cozmo SDK to receive a sequence of
actions to perform, which then passs it to the Cozmo to execute them.
"""
import time
import rospy
from libcozmo.msg import ActionMsg
import cozmo
from cozmo.util import radians

class CozmoActionNode(object):
	
	"""
    A Cozmo node class that contains the following topics:
    forklift_position, head_position, pose, camera, imu
    """
	def __init__(self):
		
		self.subscriber = rospy.Subscriber("Action", ActionMsg, self.callback, queue_size = 10)
		# while (self.subscriber.getNumPublishers() < 0):
		# 	rospy.spinOnce()
		rospy.spin()

	def callback(self, data):
		self.heading = data.heading
		self.duration = data.duration
		self.speed = data.speed
		print("I heard the following action: heading: %f duration: %f \
			speed: %f" % (self.heading, self.duration, self.speed))
		cozmo.run_program(self.move_cozmo)
		# self.move_cozmo()

	def move_cozmo(self, robot= cozmo.robot.Robot):
		
		"""
		Move cozmo with given speed, heading, and duration.

		@param robot: cozmo to execute action on
		"""

		robot.turn_in_place(
			radians(self.heading), is_absolute = True).wait_for_completed()
		
		robot.drive_wheels(self.speed, self.speed, 700, 700, duration = self.duration)
		# time.sleep(.1)

# def cozmo_run(robot= cozmo.robot):
# 	node = CozmoActionNode(robot);

if __name__ == '__main__':
	rospy.init_node("cozmo", anonymous = True)
	print("node intialized, waiting for messages..")
	try:
		# cozmo.run_program(cozmo_run)
		node = CozmoActionNode()
	except rospy.ROSInterruptException:
		pass

