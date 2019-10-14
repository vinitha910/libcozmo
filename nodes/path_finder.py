"""/////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019, Eric Pan, Vinitha Ranganeni
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
/////////////////////////////////////////////////////////////////////////////"""

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

class tempAction(object):
    """
    Temp Generic Action until pybindings for cpp version is done
    """
    def __init__(self, speed, duration, heading):
        """
        speed : double, mm/s
        duration : double, seconds
        heading : double, radians
        """
        self.speed = speed
        self.duration = duration
        self.heading = heading

class PathFinder(object):
    """
    Visualizes and executes a path given a list of generic actions
    Option to execute actions from cozmo frame or world frame

    Cozmo frame (relative path traversal):
        More control of how each action is executed but more deviation from rviz
    World frame (absolute path traversal):
        Less control of how each action is executed and much slower execution time, but less deviation from rviz
        
    """
    def __init__(self, robot, cozmo, path=None):
        """
        Initialize a path to visualize and execute

        Parameters
        ----------
        robot : cozmo.robot
            The cozmo SDK robot handle
        cozmo : cozmo skeleton mesh
        path : list of GenericActions
        waypoints : list of Waypoint objects for visualization
        poses : list of poses (x, y, theta) for execution
        time : to keep track of waypoint time, (not sure what unit this is)
        previous_heading : to keep track of additional actions to take, in radians
            This allows for smoother rviz and absolute path traversal, doesn't affect relative path traversal
        current_pos : x, y, theta (mm, mm, radians)

        For a given generic action (speed, duration, heading):
            robot will turn to face the heading and then travel
        """
        self.robot = robot
        self.cozmo = cozmo
        self.path = [] if None else path
        self.waypoints = []
        self.poses = []
        self.time = 0
        self.previous_heading = None
        self.current_pos = None
        self.init_path()

    def init_path(self, start=(0, 0, 0)):
        """
        Initialize the path if one is given, otherwise add a starting location for rviz
        """
        self.current_pos = start
        self.add(tempAction(start[0], start[1], start[2]))
        if self.path:
            for action in self.path:
                self.add(action, True)

    def add(self, action, init=False):
        """
        Adds the given action to the list of waypoints and poses for rviz and traversal respectively
        Also appends action to the path if this is called outside the init

        action : GenericAction

        """
        # Add a turn action
        if (self.previous_heading is None or action.heading != self.previous_heading):
            self.previous_heading = action.heading
            self.waypoints.append(Waypoint(self.current_pos[0] / 1000, self.current_pos[1] / 1000, action.heading, self.time + 1))
            self.poses.append((self.current_pos[0], self.current_pos[1], action.heading))
        self.update_position(action)
        self.waypoints.append(Waypoint(self.current_pos[0] / 1000, self.current_pos[1] / 1000, self.current_pos[2], self.time + 2))
        self.poses.append((self.current_pos[0], self.current_pos[1], self.current_pos[2]))
        self.time += 2
        if not init:
            self.path.append(action)

    def visualize(self):
        """
        Visualizes the path in rviz
        """
        if self.path and not rospy.is_shutdown():
            traj = self.cozmo.createInterpolatedTraj(self.waypoints);
            self.cozmo.executeTrajectory(timedelta(milliseconds=6), traj)
        else:
            print('No path to visualize')

    def execute(self, absolute=False):
        """
        Execute the path built so far

        absolute : True to use world frame traversal, false to use relative frame traversal
        """
        if self.path:
            if absolute:
                for pose in self.poses:
                    print('going to pose: ', pose[0], pose[1], pose[2])
                    self.robot.go_to_pose(pose_z_angle(pose[0], pose[1], 0, radians(pose[2]))).wait_for_completed()
            else:
                for point in self.path:
                    if point.heading:
                        self.robot.turn_in_place(radians(point.heading), is_absolute=True).wait_for_completed()
                    self.robot.drive_wheels(point.speed, point.speed, duration=point.duration)

    def update_position(self, action):
        """
        Updates the (x, y, theta) in (mm, mm, radians) given a GenericAction
        """
        self.current_pos = (self.current_pos[0] + math.cos(action.heading) * action.speed * action.duration,
                            self.current_pos[1] + math.sin(action.heading) * action.speed * action.duration,
                            action.heading)

    #TODO: Some kind of path_generation to go around object function 


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

    speed = 100
    duration = 1
    headings = [0, 0, 3 * pi / 2, pi]

    actions = []

    for heading in headings:
        actions.append(tempAction(speed, duration, heading))

    pf = PathFinder(robot, cozmo, actions)
    pf.add(tempAction(200, 1, 5))
    pf.execute(True)
    #pf.visualize()

    # path = [action1_id, action2_id]
    # for action in path: 
    #     GenericAction a = Actionspace.get_action(action)


if __name__ == '__main__':
    roscpp_initializer.roscpp_init("path_finder", [])
    rospy.init_node('path_finder')
    try:
        cozmo.run_program(cozmo_run)
    except rospy.ROSInterruptException:
        pass
