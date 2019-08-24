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

    plt.figure(0)
    plt.scatter(x, y)
    plt.plot(x[0:2], y[0:2], 'r')
    plt.plot(x[2:4], y[2:4], 'r')
    plt.scatter(corners_x, corners_y)
    
    a = [corners_x[i] for i in range(len(x)) if i % 2 == 0]
    b = [corners_y[i] for i in range(len(y)) if i % 2 == 0]
    c = [corners_x[i] for i in range(len(x)) if i % 2 != 0]
    d = [corners_y[i] for i in range(len(y)) if i % 2 != 0]
    plt.plot(corners_x[0:2], corners_y[0:2], 'b')
    plt.plot(corners_x[2:4], corners_y[2:4], 'b')
    plt.plot(a, b, 'b')
    plt.plot(c, d, 'b')
    plt.savefig('edge.png')
    plt.show()

cozmo.run_program(cozmo_program)
