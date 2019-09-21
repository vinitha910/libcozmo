#!/usr/bin/env python3
from cozmo.util import angle_z_to_quaternion, radians
from visualization_msgs.msg import Marker

class RectangularCuboid():
    """
    An object class that stores information about a custom rectangular object
    This object is composed of multiple cubes attached horizontally
    """
    def __init__(self, num_cubes, pose, side_length):
        """
        Parameters
        ----------
        pose : (x, y, theta) of the object in (mm, mm, radians)
        length : long side of the object in mm
        width : short side of the object in mm
        height : height of the object in mm
        """
        self.pose = pose
        self.length = num_cubes * side_length
        self.width = side_length
        self.height = side_length

    def update_object(self, cubes, side_length):
        """
        Updates a RectangularCuboid from given cubes and cube length
        Assumes cubes are attached horizontally

        Parameters
        ----------
        cubes : list containing LightCube objects
        side_length : length of cube side, in mm
        """
        x_positions = []
        y_positions = []

        for cube in cubes:
            x_positions.append(cube.pose.position.x)
            y_positions.append(cube.pose.position.y)

        heading = cubes[0].pose.rotation.angle_z.radians

        self.pose = (sum(x_positions) / len(cubes), sum(y_positions) / len(cubes), heading)
        self.length = len(cubes) * side_length
        self.width = side_length
        self.height = side_length

    def publish_object(self, pub, color=(0, 0.5, 0.5, 1)):
        """
        Publishes the object as a cube Marker
        """
        box_marker = Marker()
        box_marker.header.frame_id = "base_link"
        box_marker.type = Marker.CUBE

        box_marker.pose.position.x = self.pose[0] / 1000.0
        box_marker.pose.position.y = self.pose[1] / 1000.0
        box_marker.pose.position.z = self.height / 1000.0

        box_orientation = angle_z_to_quaternion(radians(self.pose[2]))
        box_marker.pose.orientation.x = box_orientation[1]
        box_marker.pose.orientation.y = box_orientation[2]
        box_marker.pose.orientation.z = box_orientation[3]
        box_marker.pose.orientation.w = box_orientation[0]

        box_marker.scale.x = self.width / 1000.0
        box_marker.scale.y = self.length / 1000.0
        box_marker.scale.z = self.width / 1000.0
        box_marker.color.r = color[0]
        box_marker.color.g = color[1]
        box_marker.color.b = color[2]
        box_marker.color.a = color[3]

        pub.publish(box_marker)

    def __str__(self):
        return "Pose: %s, Length: %s, Width: %s" % \
            (self.pose, self.length, self.width)

    def __repr__(self):
        return "CustumObject(%s, %s, %s)" % (self.pose, self.length, self.width)
