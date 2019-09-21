#!/usr/bin/env python3
from custom_object import CustomObject, create_custom_object
import rospy

import cozmo
from cozmo.util import angle_z_to_quaternion, pose_z_angle, radians
from cozmo.util import distance_mm, speed_mmps
import threading

from visualization_msgs.msg import Marker
from roscpp_initializer import roscpp_initializer
from cozmopy import Cozmo
from aikidopy import InteractiveMarkerViewer


def look_for_object(robot: cozmo.robot):
    """
    Tells Cozmo to look around for two LightCubes and constructs a CustomObject
    representing them
    """
    look_around = robot.start_behavior(
        cozmo.behavior.BehaviorTypes.LookAroundInPlace)
    try:
        cubes = robot.world.wait_until_observe_num_objects(2, cozmo.objects.LightCube, timeout=10)
    except asyncio.TimeoutError:
        print('Not enough cubes found')
    finally:
        look_around.stop()
    return create_custom_object(cubes, 45)

def publish_cozmo(pub, robot, cozmo):
    """
    Thread to publish cozmo's location asynchronously
    """
    while not rospy.is_shutdown():
        q = robot.pose.rotation
        quat = [q.q0, q.q1, q.q2, q.q3]
        cozmo.setState(
            robot.pose.position.x / 1000.0,
            robot.pose.position.y / 1000.0,
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

    t = threading.Thread(
        target=publish_cozmo,
        args=(cozmo_publisher, robot, cozmo))
    t.start()

    while not rospy.is_shutdown():
        custom_obj = look_for_object(robot)
        custom_obj.publish_object(object_publisher)
        rospy.sleep(0.001)

        # Actions to take
        robot.go_to_pose(pose_z_angle(
            custom_obj.pose[0],
            custom_obj.pose[1],
            0,
            radians(custom_obj.pose[2]))).wait_for_completed()
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
