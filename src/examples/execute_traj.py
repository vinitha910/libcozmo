#!/usr/bin/env python

import rospy
from roscpp_initializer import roscpp_initializer
from aikidopy import SkeletonMarker, InteractiveMarkerViewer
from cozmopy import Cozmo, Waypoint
from math import pi 
from datetime import timedelta
import argparse

if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument('--mesh_dir', dest='mesh_dir', required = True,
	                     help='The path to the directory containing Cozmos meshes')
	args = parser.parse_args()

	# rospy.init_node does not initialize roscpp and if roscpp is not initialized
	# instanciating ros::NodeHandle will lead to a fatal error.
	# roscpp_initializer initializes roscpp and ros::NodeHandle in the background
	roscpp_initializer.roscpp_init("load_cozmo", [])
	rospy.init_node("load_cozmo")
	rate = rospy.Rate(10)

	topicName = "dart_markers"
	baseFrameName = "map"

	if not rospy.is_shutdown():
		cozmo = Cozmo(args.mesh_dir)
		skeleton = cozmo.getCozmoSkeleton();

		print("Starting viewer. Please subscribe to the {} InteractiveMarker" 
			  " topic in Rviz".format(topicName))

		viewer = InteractiveMarkerViewer(topicName, baseFrameName)
		
		cozmo_marker = viewer.addSkeletonMarker(skeleton)
		viewer.setAutoUpdate(True);

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

		traj = cozmo.createInterpolatedTraj(waypoints);
		cozmo.executeTrajectory(timedelta(milliseconds=6), traj)	
