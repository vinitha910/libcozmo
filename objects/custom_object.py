#!/usr/bin/env python3
from cozmo.util import angle_z_to_quaternion, radians
from visualization_msgs.msg import Marker

class RectangularCuboid(object):
    """
    An object class that stores information about a custom rectangular object
    This object is composed of multiple cubes attached horizontally
    """
    def __init__(self, num_cubes, pose, side_length):
        """
        Parameters
        ----------
        num_cubes : int
            number of cubes that form this object
        pose : (x, y, theta) of the object in (mm, mm, radians)
        side_length : float
            length of a side of the cube, in mm
        """
        self.pose = pose
        self.length = num_cubes * side_length
        self.width = side_length
        self.height = side_length

        self.cube_marker = Marker()
        self.cube_marker.header.frame_id = "base_link"
        self.cube_marker.type = Marker.CUBE

        color=(0, 0.5, 0.5, 1)
        
        self.cube_marker.scale.x = self.width / 1000
        self.cube_marker.scale.y = self.length / 1000
        self.cube_marker.scale.z = self.width / 1000
        self.cube_marker.color.r = color[0]
        self.cube_marker.color.g = color[1]
        self.cube_marker.color.b = color[2]
        self.cube_marker.color.a = color[3]

    def update_object(self, cubes):
        """
        Updates the object's pose given the cubes
        Assumes cubes are attached horizontally

        Parameters
        ----------
        cubes : list containing LightCube objects
        """
        x_positions = []
        y_positions = []

        for cube in cubes:
            x_positions.append(cube.pose.position.x)
            y_positions.append(cube.pose.position.y)

        heading = cubes[0].pose.rotation.angle_z.radians

        self.pose = (sum(x_positions) / len(cubes), sum(y_positions) / len(cubes), heading)
        self.length = len(cubes) * self.width

    def publish_cube(self, publisher, color=(0, 0.5, 0.5, 1)):
        """
        Publishes the object as a cube Marker

        Parameters
        ---------
        publisher : ros publisher
        color : tuple
            (r, g, b, a) to represent the color of the cube
        """
        self.cube_marker.pose.position.x = self.pose[0] / 1000
        self.cube_marker.pose.position.y = self.pose[1] / 1000
        self.cube_marker.pose.position.z = self.height / 1000

        cube_orientation = angle_z_to_quaternion(radians(self.pose[2]))
        self.cube_marker.pose.orientation.x = cube_orientation[1]
        self.cube_marker.pose.orientation.y = cube_orientation[2]
        self.cube_marker.pose.orientation.z = cube_orientation[3]
        self.cube_marker.pose.orientation.w = cube_orientation[0]

        publisher.publish(self.cube_marker)

    def __str__(self):
        return "Pose: %s, Length: %s, Width: %s" % \
            (self.pose, self.length, self.width)

    def __repr__(self):
        return "RectangularCuboid(%s, %s, %s)" % (self.pose, self.length, self.width)
