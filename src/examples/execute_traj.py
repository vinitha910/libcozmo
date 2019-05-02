#!/usr/bin/env python

import rospy
from roscpp_initializer import roscpp_initializer
# from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
import aikidopy
from cozmopy import Cozmo, Waypoint
from math import pi 

if __name__ == "__main__":

	roscpp_initializer.roscpp_init("load_cozmo", [])
	rospy.init_node("load_cozmo")
	rate = rospy.Rate(1) # 10hz

	if not rospy.is_shutdown():
		topicName = "dart_markers"
		baseFrameName = "map"

		mesh_dir = " /home/vinitha/workspaces/cozmo_ws/src/libcozmo/src/cozmo_description/meshes"
		cozmo = Cozmo(mesh_dir)

		print("Starting viewer. Please subscribe to the %s InteractiveMarker topic in Rviz", topicName)

		viewer = aikidopy.InteractiveMarkerViewer(topicName, baseFrameName)
		viewer.addSkeleton(cozmo.getCozmoSkeleton())
		viewer.setAutoUpdate(true);

		waypoints = [
			Wapoint(0.0, 0.0, 0, 0),
		    Wapoint(0.3, 0.0, 0, 2),
		    Wapoint(0.3, 0.0, pi/2, 3),
		    Wapoint(0.3, 0.3, pi/2, 5),
		    Wapoint(0.3, 0.3, pi, 6),
		    Wapoint(0.0, 0.3, pi, 8),
		    Wapoint(0.0, 0.3, -pi/2, 9),
		    Wapoint(0.0, 0.0, -pi/2, 11),
		    Wapoint(0.0, 0.0, 0, 12),
		]

		traj = cozmo.createInterpolatedTraj(waypoints);
		cozmo.executeTrajectory(6, traj)	
