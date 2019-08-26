#!/usr/bin/env python3
import sys

import cozmo
from cozmo.util import degrees
import matplotlib.pyplot as plt

# Finds the corner coordinates of a 4 sided surface
# Assumes Cozmo starts perpendicular to an edge
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

    print(x_max - x_min, y_max - y_min)

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

def generic_detector(robot: cozmo.robot.Robot):
    print(robot.pose.rotation.angle_z.degrees)
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

    print(x_max - x_min, y_max - y_min)

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

cozmo.run_program(generic_detector)
