#!/usr/bin/env python3
import sys
import rospy

import cozmo
from cozmo.util import degrees

from visualization_msgs.msg import Marker

class CornerDetector(object):
    """
    A corner detector class that uses cozmo's built in edge detector to find
    the corners of a rectangular surface
    """
    def __init__(self, robot, threshold):
        """
        Parameters
        ----------
        robot : cozmo.robot
            The cozmo SDK robot handle
        threshold : float
            The minimum distance (mm) from the table edge Cozmo must be when
            moving around
        """
        self.robot = robot
        self.x_min = sys.maxsize
        self.x_max = -sys.maxsize - 1
        self.y_min = sys.maxsize
        self.y_max = -sys.maxsize - 1
        self.threshold = threshold
        #self.find_corners()

    def find_corners(self):
        """
        Finds the corner coordinates of a 4 sided surface with the constraint
        that cozmo needs to start perpendicular to an edge
        """
        self.robot.set_lift_height(1).wait_for_completed()

        # The order in which cozmo will turn and find edges
        for angle in [180, 91, 180, 180]:
            while not self.robot.is_cliff_detected:
                self.robot.drive_wheels(100, 100, duration=0.25)
            x_curr = self.robot.pose.position.x
            y_curr = self.robot.pose.position.y

            self.x_min = min(x_curr, self.x_min)
            self.x_max = max(x_curr, self.x_max)
            self.y_min = min(y_curr, self.y_min)
            self.y_max = max(y_curr, self.y_max)
            self.robot.drive_wheels(-100, -100, duration=1.5)
            self.robot.turn_in_place(degrees(angle)).wait_for_completed()
        self.x_max = self.x_max - self.x_min
        self.x_min = 0
        self.y_max = self.y_max - self.y_min
        self.y_min = 0

    def within_bounds(self, x_coord, y_coord):
        """
        Given x, y coordinates (in mm), returns if the coordinates are within bounds
        """
        return self.x_min + self.threshold <= y_coord <= self.x_max - self.threshold and \
               self.y_min + self.threshold <= x_coord <= self.y_max - self.threshold

    def publish_plane(self, pub, height, boundary=False, color=(0, 0.5, 0.5)):
        """
        Publish the surface found based on corners of the surface

        Parameters
        ----------
        pub : ros publisher
        height : float
            The height of the surface
        boundary : bool
            False to publish the surface, True to publish the boundary of the surface
        color : tuple
            (r, g, b) values for the color of the surface
        All position and scale units are in mm
        X and Y are switched because of the way cozmo moves in rviz
        """
        plane_marker = Marker()
        plane_marker.header.frame_id = "base_link"
        plane_marker.type = Marker.CUBE

        plane_marker.pose.position.x = self.y_max / 2000
        plane_marker.pose.position.y = self.x_max / 2000
        plane_marker.pose.position.z = height / 1000

        plane_marker.pose.orientation.x = 0
        plane_marker.pose.orientation.y = 0
        plane_marker.pose.orientation.z = 0
        plane_marker.pose.orientation.w = 1

        x_scale = (self.x_max - self.x_min) / 1000
        y_scale = (self.y_max - self.y_min) / 1000
        plane_marker.scale.y = x_scale if not boundary else x_scale - 2 * self.threshold / 1000
        plane_marker.scale.x = y_scale if not boundary else y_scale - 2 * self.threshold / 1000
        plane_marker.scale.z = 1 / 1000

        plane_marker.color.r = color[0]
        plane_marker.color.g = color[1]
        plane_marker.color.b = color[2]
        plane_marker.color.a = 1

        pub.publish(plane_marker)

def cozmo_run(robot: cozmo.robot):
    detector = CornerDetector(robot, 10)
    plane_publisher = rospy.Publisher('plane_marker', Marker, queue_size=10)
    boundary_publisher = rospy.Publisher('boundary_marker', Marker, queue_size=10)

    while not rospy.is_shutdown():
        detector.publish_plane(plane_publisher, 0, color=(1, 0.5, 0))
        detector.publish_plane(boundary_publisher, 2, True)

if __name__ == '__main__':
    rospy.init_node("edge_detection")
    try:
        cozmo.run_program(cozmo_run)
    except rospy.ROSInterruptException:
        pass
