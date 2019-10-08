#!/usr/bin/env python3
import rospy

import cozmo
from cozmo.util import degrees, pose_z_angle, radians
import threading
import math
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

    # path = [action1_id, action2_id]
    # for action in path: 
    #     GenericAction a = Actionspace.get_action(action)

    # Create a path that is in the shape of a j
    speed = 100
    duration = 1
    headings = [0, 0, 3 * pi / 2, pi]

    current_pos = (0, 0, 0)
    poses = []
    poses.append(current_pos)
    waypoints = []
    waypoints.append(Waypoint(current_pos[0], current_pos[1], current_pos[2], 0))

    for i in range(len(headings))[1:]:
        current_pos = generic_action_to_xytheta(current_pos, speed, duration, headings[i])
        poses.append(current_pos)
        waypoints.append(Waypoint(current_pos[0], current_pos[1], current_pos[2], i*2))

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

    print(waypoints)
    print(poses)
    traj = cozmo.createInterpolatedTraj(waypoints);
    cozmo.executeTrajectory(timedelta(milliseconds=6), traj)

    #for waypoint in waypoints:
    #    robot.go_to_pose(pose_z_angle(waypoint[0], waypoint[1], 0, radians(waypoint[2]))).wait_for_completed()
    #path_finder = PathFinder(robot)

def generic_action_to_xytheta(current_pos, speed, duration, heading):
    # x, in mm
    return (current_pos[0] + math.cos(heading) * speed * duration,
            current_pos[1] + math.sin(heading) * speed * duration,
            heading)


if __name__ == '__main__':
    roscpp_initializer.roscpp_init("path_finder", [])
    rospy.init_node('path_finder')
    try:
        cozmo.run_program(cozmo_run)
    except rospy.ROSInterruptException:
        pass

# Could potentially use drive_wheels, rotate_in_place and calculate waypoint movement instead of go_to_pose,
# but need to test which is better
