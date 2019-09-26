#!/usr/bin/env python3
import sys
sys.path.append('../objects')
from edge_detection import CornerDetector
from custom_object import RectangularCuboid
import rospy

import cozmo
from cozmo.util import angle_z_to_quaternion, pose_z_angle, radians
from cozmo.util import distance_mm, speed_mmps
import threading

from visualization_msgs.msg import Marker
from roscpp_initializer import roscpp_initializer
from cozmopy import Cozmo
from aikidopy import InteractiveMarkerViewer


def look_for_object(robot: cozmo.robot, timeout=2):
    """
    Tells Cozmo to look around for two LightCubes and constructs a CustomObject
    representing them
    """
    done = False
    while not done:
        try:
            look_around = robot.start_behavior(
                cozmo.behavior.BehaviorTypes.LookAroundInPlace)
            cubes = robot.world.wait_until_observe_num_objects(2, cozmo.objects.LightCube, timeout=timeout)
            if cubes:
                done = True
            else:
                look_around.stop()
                input("Cozmo can't find any cubes, please reset cozmo and cubes and then press Enter to continue")
        except asyncio.TimeoutError:
            continue
        finally:
            look_around.stop()
    cube = RectangularCuboid(2, (0, 0, 0), 45)
    cube.update_object(cubes)
    return cube

def publish_cozmo(pub, robot, cozmo):
    """
    Thread to publish cozmo's location asynchronously
    """
    while not rospy.is_shutdown():
        q = robot.pose.rotation
        quat = [q.q0, q.q1, q.q2, q.q3]
        cozmo.setState(
            (robot.pose.position.x - 45) / 999.0,
            (robot.pose.position.y + 60) / 1000.0,
            quat)
        rospy.sleep(0.001)

def cozmo_run(robot: cozmo.robot):
    topicName = "cozmo_model"
    baseFrameName = "base_link"

    if not rospy.is_shutdown():
        cozmo = Cozmo("/home/bubbletea/cozmo_ws/src/libcozmo/src/cozmo_description/meshes")
        skeleton = cozmo.getCozmoSkeleton();

        viewer = InteractiveMarkerViewer(topicName, baseFrameName)
        cozmo_marker = viewer.addSkeleton(skeleton)
        viewer.setAutoUpdate(True);

    cozmo_publisher = rospy.Publisher('cozmo_marker', Marker, queue_size=10)
    object_publisher = rospy.Publisher('object_marker', Marker, queue_size=10)
    plane_publisher = rospy.Publisher('plane_marker', Marker, queue_size=10)
    boundary_publisher = rospy.Publisher('boundary_marker', Marker, queue_size=10)

    t = threading.Thread(
        target=publish_cozmo,
        args=(cozmo_publisher, robot, cozmo))
    t.start()

    look_for_duration = float(input('How long to look for cubes? '))
    threshold = float(input('Boundary threshold? '))
    detector = CornerDetector(robot, threshold)
    input('Reset Cozmo to the bottom left of the surface for alignment')

    while not rospy.is_shutdown():
        detector.publish_plane(plane_publisher, 0, color=(1, 0.5, 0))
        detector.publish_plane(boundary_publisher, 2, True)

        custom_obj = look_for_object(robot, look_for_duration)
        custom_obj.publish_cube(object_publisher)
        rospy.sleep(0.001)

        # Part 1 of oo action: getting to object
        robot.go_to_pose(pose_z_angle(
            custom_obj.pose[0],
            custom_obj.pose[1],
            0,
            radians(custom_obj.pose[2]))).wait_for_completed()
        # Part 2 of oo action: pushing object
        # robot.drive_wheels(speed, speed, duration=duration)
        
        x_coord = robot.pose.position.x
        y_coord = robot.pose.position.y
        
        # check if cozmo is within bounds:
        if not detector.within_bounds(x_coord, y_coord):
            print(detector.y_max, detector.x_max)
            print(x_coord, y_coord)
            input('Cozmo is out of bounds, please reset and press Enter to continue')
        else:
            print('within bounds')
            # update model 

        # Optional command that tells cozmo to move back a bit to detect cubes better
        robot.drive_straight(
            distance_mm(-100),
            speed_mmps(100)).wait_for_completed()

if __name__ == '__main__':
    roscpp_initializer.roscpp_init("custom_object", [])
    rospy.init_node("custom_object")
    rate = rospy.Rate(10)

    try:
        cozmo.run_program(cozmo_run)
    except rospy.ROSInterruptException:
        pass
