#!/usr/bin/env python3
import cozmo
from cozmo.util import degrees
import matplotlib.pyplot as plt

# Finds the corner coordinates of a 4 sided surface
# Assumes Cozmo starts perpendicular to an edge
def cozmo_program(robot: cozmo.robot.Robot):
    x = []
    y = []
    corners = []
    robot.set_lift_height(1).wait_for_completed()

    for angle in [180, 90, 180, 180]:
        while not robot.is_cliff_detected:
            robot.drive_wheels(100, 100, duration=0.25)
        x.append(robot.pose.position.x)
        y.append(robot.pose.position.y)
        robot.drive_wheels(-100, -100, duration=1.5)
        robot.turn_in_place(degrees(angle)).wait_for_completed()

    corners_x = [x[1], x[0], x[1], x[0]]
    corners_y = [y[2], y[2], y[3], y[3]]
    corners.append((x[1], y[2]))
    corners.append((x[0], y[2]))
    corners.append((x[1], y[3]))
    corners.append((x[0], y[3]))

    plt.figure(0)
    plt.scatter(x, y)
    plt.figure(1)
    plt.scatter(corners_x, corners_y)
    plt.show()

cozmo.run_program(cozmo_program)
