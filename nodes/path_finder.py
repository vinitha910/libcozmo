#!/usr/bin/env python3
import rospy

import cozmo
from cozmo.util import degrees, pose_z_angle, radians
import threading
from math import pi
from datetime import timedelta

from roscpp_initializer import roscpp_initializer
from cozmopy import Cozmo, Waypoint
from aikidopy import InteractiveMarkerViewer

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

def update_cozmo(robot, cozmo):
    """
    Thread to update cozmo's location
    """
    while not rospy.is_shutdown():
        q = robot.pose.rotation
        quat = [q.q0, q.q1, q.q2, q.q3]
        cozmo.setState(
            robot.pose.position.x / 1000,
            robot.pose.position.y / 1000,
            quat)
        rospy.sleep(0.001)

def cozmo_run(robot: cozmo.robot):
    topicName = "cozmo_model"
    baseFrameName = "base_link"
    
    if not rospy.is_shutdown():
        cozmo = Cozmo("/home/bubbletea/cozmo_ws/src/libcozmo/src/cozmo_description/meshes")
        skeleton = cozmo.getCozmoSkeleton()
        
        viewer = InteractiveMarkerViewer(topicName, baseFrameName)
        cozmo_marker = viewer.addSkeleton(skeleton)
        viewer.setAutoUpdate(True)

    t = threading.Thread(
        target = update_cozmo,
        args=(robot, cozmo))
    t.start()

    waypoints = [
            Waypoint(0.0, 0.0, 0, 0),
        Waypoint(0.3, 0.0, 0, 2),
        Waypoint(0.3, 0.0, pi/2, 3),
        Waypoint(0.3, 0.3, pi/2, 5),
        Waypoint(0.3, 0.3, pi, 6),
        Waypoint(0.0, 0.3, pi, 8),
        Waypoint(0.0, 0.3, -pi/2, 9),
        Waypoint(0.0, 0.0, -pi/2, 11),
        Waypoint(0.0, 0.0, 0, 12),
    ]

    #traj = cozmo.createInterpolatedTraj(waypoints);
    # cozmo.executeTrajectory(timedelta(milliseconds=6), traj)

    for waypoint in waypoints:
        print([method_name for method_name in dir(waypoint)
                      if callable(getattr(waypoint, method_name))])
        robot.go_to_pose(pose_z_angle(waypoint[0], waypoint[1], 0, radians(waypoint[2]))).wait_for_completed()
    #path_finder = PathFinder(robot)



if __name__ == '__main__':
    roscpp_initializer.roscpp_init("path_finder", [])
    rospy.init_node('path_finder')
    try:
        cozmo.run_program(cozmo_run)
    except rospy.ROSInterruptException:
        pass

# Could potentially use drive_wheels, rotate_in_place and calculate waypoint movement instead of go_to_pose,
# but need to test which is better
