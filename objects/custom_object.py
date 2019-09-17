#!/usr/bin/env python3
import rospy

import cozmo
from cozmo.util import angle_z_to_quaternion, pose_z_angle, radians, distance_mm, speed_mmps
import math
import threading

from visualization_msgs.msg import Marker
from roscpp_initializer import roscpp_initializer
from cozmopy import Cozmo
from aikidopy import InteractiveMarkerViewer

class CustomObject():
    """
    A custom object class that stores information about the object
    """
    def __init__(self, pose, length, width):
        """
        Parameters
        ----------
        pose : (x, y, theta) of the object in (mm, mm, radians)
        length : long side of the object in mm
        width : short side of the object in mm
        """
        self.pose = pose
        self.length = length
        self.width = width

    def __str__(self):
        return "Pose: %s, Length: %s, Width: %s" % \
            (self.pose, self.length, self.width)

    def __repr__(self):
        return "CustumObject(%s, %s, %s)" % (self.pose, self.length, self.width)

# Returns if two cubes are attached
def are_attached(cube1, cube2, threshold):
    x1 = cube1.pose.position.x
    y1 = cube1.pose.position.y
    x2 = cube2.pose.position.x
    y2 = cube2.pose.position.y

    return math.sqrt(abs(x2 - x1) ** 2 +  abs(y2 - y1) ** 2) < threshold + 5

def create_custom_object(cubes, cube_len):
    x1 = cubes[0].pose.position.x
    y1 = cubes[0].pose.position.y
    x2 = cubes[1].pose.position.x
    y2 = cubes[1].pose.position.y
    heading = cubes[0].pose.rotation.angle_z.radians

    return CustomObject(((x1+ x2) / 2, (y1 + y2) / 2, heading), 2 * cube_len, cube_len)

def look_for_object(robot: cozmo.robot):
    look_around = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
    try:
        cubes = robot.world.wait_until_observe_num_objects(2, cozmo.objects.LightCube, timeout=10)
        #cube = robot.world.wait_for_observed_light_cube(timeout=10)
    except asyncio.TimeoutError:
        print('Not enough cubes found')
    finally:
        look_around.stop()
    return create_custom_object(cubes, 45)

def cozmo_thread(pub, robot, cozmo):
    while not rospy.is_shutdown():
        publish_cozmo(pub, robot, cozmo)
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

    # Can't set attribute
    #reset_pose = pose_z_angle(0, 0, 0, radians(0))
    #robot.pose = reset_pose

    t = threading.Thread(target=cozmo_thread, args=(cozmo_publisher, robot, cozmo))
    t.start()

    while not rospy.is_shutdown():
        custom_obj = look_for_object(robot)

        publish_cozmo(cozmo_publisher, robot, cozmo)
        publish_object(object_publisher, custom_obj)
        rospy.sleep(0.001)
        
        robot.go_to_pose(pose_z_angle(custom_obj.pose[0], custom_obj.pose[1], 0, radians(custom_obj.pose[2]))).wait_for_completed()
        robot.drive_straight(distance_mm(-100), speed_mmps(100)).wait_for_completed()


def publish_cozmo(pub, robot, cozmo):
    q = robot.pose.rotation
    quat = [q.q0, q.q1, q.q2, q.q3]
    cozmo.setState(robot.pose.position.x / 1000.0, robot.pose.position.y / 1000.0, quat)

def publish_object(pub, custom_obj):
    box_marker = Marker()
    box_marker.header.frame_id = "base_link"
    box_marker.type = Marker.CUBE

    box_marker.pose.position.x = custom_obj.pose[0] / 1000.0
    box_marker.pose.position.y = custom_obj.pose[1] / 1000.0
    box_marker.pose.position.z = 35 / 1000.0
    # conversion not necessary anymore 
    box_orientation = angle_z_to_quaternion(radians(custom_obj.pose[2]))
    print(box_orientation)
    box_marker.pose.orientation.x = box_orientation[1]
    box_marker.pose.orientation.y = box_orientation[2]
    box_marker.pose.orientation.z = box_orientation[3]
    box_marker.pose.orientation.w = box_orientation[0]

    box_marker.scale.x = custom_obj.width / 1000.0
    box_marker.scale.y = custom_obj.length / 1000.0
    box_marker.scale.z = custom_obj.width / 1000.0
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0
    
    pub.publish(box_marker)

if __name__ == '__main__':
    roscpp_initializer.roscpp_init("custom_object", [])
    rospy.init_node("custom_object")
    rate = rospy.Rate(10)

    try:
        cozmo.run_program(cozmo_run)
    except rospy.ROSInterruptException:
        pass
    parser = argparse.ArgumentParser()
    parser.add_argument('--mesh_dir', dest='mesh_dir', required = True,
                         help='The path to the directory containing Cozmos meshes')
    args = parser.parse_args()

    # rospy.init_node does not initialize roscpp and if roscpp is not initialized
    # instanciating ros::NodeHandle will lead to a fatal error.
    # roscpp_initializer initializes roscpp and ros::NodeHandle in the background

