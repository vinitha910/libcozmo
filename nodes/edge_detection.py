#!/usr/bin/env python3
import rospy
import sys

import cozmo
from cozmo.util import degrees
import matplotlib.pyplot as plt

from visualization_msgs.msg import Marker

# Finds the corner coordinates of a 4 sided surface
# Assumes Cozmo starts perpendicular to an edge
# Cozmo will visit each edge once
def perpendicular_detector(robot: cozmo.robot.Robot):
    x_min = sys.maxsize
    x_max = -sys.maxsize - 1
    y_min = sys.maxsize
    y_max = -sys.maxsize - 1
    x = []
    y = []

    corners = []
    robot.set_lift_height(1).wait_for_completed()

    for angle in [180, 90, 180, 180]:
        while not robot.is_cliff_detected:
            robot.drive_wheels(100, 100, duration=0.25)
        x_curr = robot.pose.position.x
        y_curr = robot.pose.position.y
        x.append(x_curr)
        y.append(y_curr)

        x_min = min(x_curr, x_min)
        x_max = max(x_curr, x_max)
        y_min = min(y_curr, y_min)
        y_max = max(y_curr, y_max)
        robot.drive_wheels(-100, -100, duration=1.5)
        robot.turn_in_place(degrees(angle)).wait_for_completed()

    plt.figure(0)
    plt.scatter(x, y)
    plt.plot(x[0:2], y[0:2], 'r')
    plt.plot(x[2:4], y[2:4], 'r')

    plt.scatter([x_min, x_min, x_max, x_max], [y_min, y_max, y_min, y_max])
    plt.plot([x_min, x_max], [y_min, y_min], 'b')
    plt.plot([x_min, x_max], [y_max, y_max], 'b')
    plt.plot([x_min, x_min], [y_min, y_max], 'b')
    plt.plot([x_max, x_max], [y_min, y_max], 'b')

    plt.savefig('edge.png')
    plt.show()

# Finds the corner coordinates of a 4 sided surface
# Cozmo will continuously find coordinates of the edges of the surface
# until new coordinates are no longer expanding the known dimensions
def generic_detector(robot: cozmo.robot.Robot):
    x_min = sys.maxsize
    x_max = -sys.maxsize - 1
    y_min = sys.maxsize
    y_max = -sys.maxsize - 1

    x = []
    y = []

    corners = []
    robot.set_lift_height(1).wait_for_completed()
    done = False

    while not done:
        while not robot.is_cliff_detected:
            robot.drive_wheels(100, 100, duration=0.25)
        x_curr = robot.pose.position.x
        y_curr = robot.pose.position.y
        x.append(x_curr)
        y.append(y_curr)
        robot.drive_wheels(-100, -100, duration=1.5)

        robot.turn_in_place(degrees(91)).wait_for_completed()

        x_min = min(x_curr, x_min)
        x_max = max(x_curr, x_max)
        y_min = min(y_curr, y_min)
        y_max = max(y_curr, y_max)

        if x_min != x_curr and x_max != x_curr and y_min != y_curr and y_max != y_curr:
            done = True

    plt.figure(0)
    plt.scatter(x, y)
    plt.plot(x, y, 'r')

    plt.scatter([x_min, x_min, x_max, x_max], [y_min, y_max, y_min, y_max])
    plt.plot([x_min, x_max], [y_min, y_min], 'b')
    plt.plot([x_min, x_max], [y_max, y_max], 'b')
    plt.plot([x_min, x_min], [y_min, y_max], 'b')
    plt.plot([x_max, x_max], [y_min, y_max], 'b')
    plt.savefig('generic_edge_detector')

    plt.show()

# Publish the surface found based on the 4 corners
def publish_plane(pub, dimensions):
    plane_marker = Marker()
    plane_marker.header.frame_id = "base_link"
    plane_marker.type = Marker.CUBE

    plane_marker.pose.position.x = 0
    plane_marker.pose.position.y = 0
    plane_marker.pose.position.z = 0

    plane_marker.pose.orientation.x = 0
    plane_marker.pose.orientation.y = 0
    plane_marker.pose.orientation.z = 0
    plane_marker.pose.orientation.w = 1

    plane_marker.scale.x = dimensions[0] / 1000.0
    plane_marker.scale.y = dimensions[1] / 1000.0
    plane_marker.scale.z = 1 / 1000.0

    plane_marker.color.r = 0.0
    plane_marker.color.g = 0.5
    plane_marker.color.b = 0.5
    plane_marker.color.a = 1.0

    pub.publish(plane_marker)

if __name__ == '__main__':
    rospy.init_node("edge_detection")
    try:
        cozmo.run_program(perpendicular_detector)
    except rospy.ROSInterruptException:
        pass
