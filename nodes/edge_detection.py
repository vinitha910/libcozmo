#!/usr/bin/env python3
import rospy
import sys

import cozmo
from cozmo.util import degrees
import matplotlib.pyplot as plt

from visualization_msgs.msg import Marker

class CornerDetector(object):
    """
    A corner detector class that uses cozmo's sensor to map the dimensions of a
    rectangular surface
    """
    def __init__(self, cozmo, threshold):
        """
        Parameters
        ----------
        cozmo : cozmo.robot
            the cozmo SDK robot handle
        threshold : float
            the distance away from an edge to be within bounds, in mm
        """
        self.cozmo = cozmo
        self.x_min = sys.maxsize
        self.x_max = -sys.maxsize - 1
        self.y_min = sys.maxsize
        self.y_max = -sys.maxsize - 1
        self.threshold = threshold

    def find_corners(self):
        """
        Finds the corner coordinates of a 4 sided surface with the constraint
        that cozmo needs to start perpendicular to an edge
        """
        x = []
        y = []

        corners = []
        self.cozmo.set_lift_height(1).wait_for_completed()

        # The order in which cozmo will turn and find edges
        for angle in [180, 90, 180, 180]:
            while not self.cozmo.is_cliff_detected:
                self.cozmo.drive_wheels(100, 100, duration=0.25)
            x_curr = self.cozmo.pose.position.x
            y_curr = self.cozmo.pose.position.y
            x.append(x_curr)
            y.append(y_curr)

            self.x_min = min(x_curr, self.x_min)
            self.x_max = max(x_curr, self.x_max)
            self.y_min = min(y_curr, self.y_min)
            self.y_max = max(y_curr, self.y_max)
            self.cozmo.drive_wheels(-100, -100, duration=1.5)
            self.cozmo.turn_in_place(degrees(angle)).wait_for_completed()

    def within_bounds(self, x, y):
        """
        Given x, y coordinates (in mm), returns if the coordinates are within bounds
        """
        return self.x_min + threshold <= x <= self.x_max - threshold and \
               self.y_min + threshold <= y <= self.y_max - threshold

    def publish_plane(self, pub, x, y, z, color=(0, 0.5, 0.5)):
        """
        Publish the surface found based on x and y dimensions

        Parameters
        ----------
        pub : ros publisher
        x : float
            The x dimension of the surface
        y : float
            The y dimension of the surface
        z : float
            The height of the surface
        All position and scale units are in mm
        """
        plane_marker = Marker()
        plane_marker.header.frame_id = "base_link"
        plane_marker.type = Marker.CUBE

        plane_marker.pose.position.x = 0
        plane_marker.pose.position.y = 0
        plane_marker.pose.position.z = z / 1000

        plane_marker.pose.orientation.x = 0
        plane_marker.pose.orientation.y = 0
        plane_marker.pose.orientation.z = 0
        plane_marker.pose.orientation.w = 1

        plane_marker.scale.x = x / 1000.0
        plane_marker.scale.y = y / 1000.0
        plane_marker.scale.z = 1 / 1000.0

        plane_marker.color.r = color[0]
        plane_marker.color.g = color[1]
        plane_marker.color.b = color[2]
        plane_marker.color.a = 1

        pub.publish(plane_marker)

def cozmo_run(robot: cozmo.robot):
    detector = CornerDetector(robot, 10)
    plane_publisher = rospy.Publisher('plane_marker', Marker, queue_size=10)
    boundary_publisher = rospy.Publisher('boundary_marker', Marker, queue_size=10)

    detector.find_corners()
    x = detector.x_max - detector.x_min
    y = detector.y_max - detector.y_min
    while not rospy.is_shutdown():
        detector.publish_plane(plane_publisher, x, y, 0, color=(1, 0.5, 0))
        detector.publish_plane(boundary_publisher, x - 2 * detector.threshold, y - 2 * detector.threshold, 2)

if __name__ == '__main__':
    rospy.init_node("edge_detection")
    try:
        cozmo.run_program(cozmo_run)
    except rospy.ROSInterruptException:
        pass
