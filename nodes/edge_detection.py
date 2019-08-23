#!/usr/bin/env python3
import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps
import matplotlib.pyplot as plt

def cozmo_program(robot: cozmo.robot.Robot):
    # robot.is_cliff_detected:
    x = []
    y = []
    counter = 0
    done = False
    robot.set_lift_height(1).wait_for_completed()
    while not done:
        robot.drive_wheels(25, 25, duration=1)
        if robot.is_cliff_detected:
            x.append(robot.pose.position.x)
            y.append(robot.pose.position.y)
            robot.drive_wheels(-100, -100, duration=1.5)
            robot.turn_in_place(degrees(90)).wait_for_completed()
            counter += 1
        if counter == 10:
            done = True
         
    plt.figure()
    plt.scatter(x, y)
    plt.show()
cozmo.run_program(cozmo_program)
