#!/usr/bin/env python

import rospy
from roscpp_initializer import roscpp_initializer
from aikidopy import SkeletonMarker, InteractiveMarkerViewer
from cozmopy import Cozmo
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
              " topic in Rviz \n".format(topicName))

        viewer = InteractiveMarkerViewer(topicName, baseFrameName)
        cozmo_marker = viewer.addSkeleton(skeleton)
        viewer.setAutoUpdate(True);

        input_str = ""
        input_val = 0
        while input_val != -1.0:
            input_str = raw_input("\nEnter forklift position (0-0.86 radians, -1 to quit): ")
            try:
                input_val = float(input_str)
                print input_val
            except ValueError as verr:
                print('Please enter a valid float value\n')
                continue

            if input_val == -1.0:
                break

            elif (input_val > 0.86 or input_val < 0):
                print('This value exceeds the joint limits, please enter valid value\n')
                continue
            
            cozmo.setForkliftPosition(input_val);
