#!/usr/bin/env python3
import rospy

import cozmo

class PathFinder(object):

    def __init__(self, robot):
        self.robot = robot

    def find_path(self, steps, goal):
        """
        steps : int
            how many steps to take to get to goal
        """

    def move_forward(self, speed=50, duration=1):
        robot.drive_wheels(speed, speed, duration=duration)

    def rotate(self, angle):
        """
        Turns the robot
        """
        robot.turn_in_place(degrees(angle)).wait_for_completed()


def cozmo_run(robot: cozmo.robot):
    path_finder = PathFinder(robot)



if __name__ == '__main__':
    rospy.init_node('path_finder')
    try:
        cozmo.run_program(cozmo_run)
    except rospy.ROSInterruptException:
        pass
