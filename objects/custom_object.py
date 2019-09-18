#!/usr/bin/env python3
from cozmo.util import angle_z_to_quaternion, radians

from visualization_msgs.msg import Marker

class CustomObject():
    """
    An object class that stores information about a custom rectangular object
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

    def publish_object(self, pub):
        """
        Publishes the object as a cube Marker
        """
        box_marker = Marker()
        box_marker.header.frame_id = "base_link"
        box_marker.type = Marker.CUBE

        box_marker.pose.position.x = self.pose[0] / 1000.0
        box_marker.pose.position.y = self.pose[1] / 1000.0
        box_marker.pose.position.z = 35 / 1000.0

        box_orientation = angle_z_to_quaternion(radians(self.pose[2]))
        box_marker.pose.orientation.x = box_orientation[1]
        box_marker.pose.orientation.y = box_orientation[2]
        box_marker.pose.orientation.z = box_orientation[3]
        box_marker.pose.orientation.w = box_orientation[0]

        box_marker.scale.x = self.width / 1000.0
        box_marker.scale.y = self.length / 1000.0
        box_marker.scale.z = self.width / 1000.0
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0

        pub.publish(box_marker)

    def __str__(self):
        return "Pose: %s, Length: %s, Width: %s" % \
            (self.pose, self.length, self.width)

    def __repr__(self):
        return "CustumObject(%s, %s, %s)" % (self.pose, self.length, self.width)

def create_custom_object(cubes, cube_len):
    """
    Creates a CustomObject from given cubes and cube length
    Assumes cubes are attached

    Parameters
    ----------
    cubes : list containing two LightCube objects
    cube_len : length of cube side, in mm

    returns a CustomObject
    """
    x1 = cubes[0].pose.position.x
    y1 = cubes[0].pose.position.y
    x2 = cubes[1].pose.position.x
    y2 = cubes[1].pose.position.y
    heading = cubes[0].pose.rotation.angle_z.radians

    return CustomObject(((x1+ x2) / 2,
        (y1 + y2) / 2, heading),
        2 * cube_len, cube_len)
