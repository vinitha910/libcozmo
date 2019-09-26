#!/usr/bin/env python3
import sys
sys.path.append('../objects')
from edge_detection import CornerDetector
from custom_object import RectangularCuboid
import rospy

import cozmo
from cozmo.util import angle_z_to_quaternion, pose_z_angle, radians, degrees
from cozmo.util import distance_mm, speed_mmps
import threading

from visualization_msgs.msg import Marker
from roscpp_initializer import roscpp_initializer
from cozmopy import Cozmo, ObjectOrientedActionSpace
from aikidopy import InteractiveMarkerViewer

import random


def look_for_object(robot: cozmo.robot, cube, timeout=2):
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
                val = input("Cozmo can't find any cubes, please reset cozmo and cubes and then press Enter to continue")
                if val == '-1':
                    sys.exit(0)
        except asyncio.TimeoutError:
            continue
        finally:
            look_around.stop()
    # cube = RectangularCuboid(2, (0, 0, 0), 45)
    # cube.update_object(cubes)
    return cubes

def publish_cozmo(pub, robot, cozmo):
    """
    Thread to publish cozmo's location asynchronously
    """
    while not rospy.is_shutdown():
        q = robot.pose.rotation
        quat = [q.q0, q.q1, q.q2, q.q3]
        cozmo.setState(
            (robot.pose.position.x - 45) / 1000.0,
            (robot.pose.position.y + 60) / 1000.0,
            quat)
        rospy.sleep(0.001)

def cozmo_run(robot: cozmo.robot):
    topicName = "cozmo_model"
    baseFrameName = "base_link"
    custom_obj = RectangularCuboid(2, (0, 0, 0), 45)

    if not rospy.is_shutdown():
        cozmo = Cozmo("/home/bubbletea/cozmo_ws/src/libcozmo/src/cozmo_description/meshes")
        skeleton = cozmo.getCozmoSkeleton();
        fake_cozmo = Cozmo("/home/bubbletea/cozmo_ws/src/libcozmo/src/cozmo_description/meshes")
        fake_skeleton = cozmo.getCozmoSkeleton();

        viewer = InteractiveMarkerViewer(topicName, baseFrameName)
        cozmo_marker = viewer.addSkeleton(skeleton)
        fake_cozmo_marker = viewer.addSkeleton(fake_skeleton)
        viewer.setAutoUpdate(True);

    cozmo_publisher = rospy.Publisher('cozmo_marker', Marker, queue_size=10)
    object_publisher = rospy.Publisher('object_marker', Marker, queue_size=10)
    plane_publisher = rospy.Publisher('plane_marker', Marker, queue_size=10)
    boundary_publisher = rospy.Publisher('boundary_marker', Marker, queue_size=10)
    cozmo_start_publisher = rospy.Publisher('cozmo_start_marker', Marker, queue_size=10)

    t = threading.Thread(
        target=publish_cozmo,
        args=(cozmo_publisher, robot, cozmo))
    t.start()

    #look_for_duration = float(input('How long to look for cubes? '))
    #threshold = float(input('Boundary threshold? '))
    #detector = CornerDetector(robot, threshold)
    detector = CornerDetector(robot, 60)
    # Manually set dimensions
    detector.x_min, detector.x_max, detector.y_min, detector.y_max = 0, 609, 0, 1524
    input('Reset Cozmo to the bottom left of the surface for alignment')

    robot.turn_in_place(degrees(30)).wait_for_completed()
    robot.drive_wheels(100, 100, duration=2)

    while not rospy.is_shutdown():
        # detector.publish_plane(plane_publisher, 0, color=(1, 0.5, 0))
        # detector.publish_plane(boundary_publisher, 2, True)

        custom_obj.update_object(look_for_object(robot, custom_obj, 5))
        custom_obj.publish_cube(object_publisher, color=(1, 0.2, 0, 1))
        rospy.sleep(0.001)

        actionspace = ObjectOrientedActionSpace([10, 100], [44, 89], [30, 30], [50, 95], 3)

        state = cozmo.createState(custom_obj.pose[0], custom_obj.pose[1], custom_obj.pose[2])
        action_id = random.choice([i for i in range(actionspace.size())])
        action = actionspace.get_generic_to_object_oriented_action(action_id, state)

        # publishing where we want cozmo to go
        publish_cozmo_start(cozmo_start_publisher, [action.start_pose()[0], action.start_pose()[1], angle_z_to_quaternion(radians(action.start_pose()[2]))])

        # # Part 1 of oo action: getting to object
        robot.go_to_pose(pose_z_angle(action.start_pose()[0],
            action.start_pose()[1],
            0,
            radians(action.start_pose()[2]))).wait_for_completed()

        # # Part 2 of oo action: pushing object
        robot.drive_wheels(action.speed(), action.speed(), duration=3)
        
        # x_coord = robot.pose.position.x
        # y_coord = robot.pose.position.y
        
        # print('Cozmo position: ', (x_coord, y_coord), 'table coords: ', (detector.y_max, detector.x_max, '\n'))
        # # check if cozmo is within bounds:
        # if not detector.within_bounds(x_coord, y_coord):
        #     input('Cozmo is out of bounds, please reset and press Enter to continue')
        # else:
        #     print('within bounds')
        #     # update model 

        # # Optional command that tells cozmo to move back a bit to detect cubes better
        # robot.drive_straight(
        #     distance_mm(-100),
        #     speed_mmps(100)).wait_for_completed()

def publish_cozmo_start(publisher, cozmo_pose, color=(0, 0, 0, 1)):
    cozmo_marker = Marker()
    cozmo_marker.header.frame_id = "base_link"
    cozmo_marker.type = Marker.CUBE

    cozmo_marker.pose.position.x = (cozmo_pose[0]) / 1000
    cozmo_marker.pose.position.y = (cozmo_pose[1] + 45) / 1000
    cozmo_marker.pose.position.z = 0

    cozmo_marker.pose.orientation.x = cozmo_pose[2][1]
    cozmo_marker.pose.orientation.y = cozmo_pose[2][2]
    cozmo_marker.pose.orientation.z = cozmo_pose[2][3]
    cozmo_marker.pose.orientation.w = cozmo_pose[2][0]

    cozmo_marker.scale.y = 45 / 1000
    cozmo_marker.scale.x = 89 / 1000
    cozmo_marker.scale.z = 45 / 1000

    cozmo_marker.color.r = color[0]
    cozmo_marker.color.g = color[1]
    cozmo_marker.color.b = color[2]
    cozmo_marker.color.a = 1

    publisher.publish(cozmo_marker)


if __name__ == '__main__':
    roscpp_initializer.roscpp_init("custom_object", [])
    rospy.init_node("custom_object")
    rate = rospy.Rate(10)

    try:
        cozmo.run_program(cozmo_run)
    except rospy.ROSInterruptException:
        pass
