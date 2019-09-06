#!/usr/bin/env python3
import rospy

import cozmo
from cozmo.util import pose_z_angle, radians
import math

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

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


def cozmo_run(robot: cozmo.robot):
    num_cubes = 2
    cube_len = 45

    look_around = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
    try:
        cubes = robot.world.wait_until_observe_num_objects(num_cubes, cozmo.objects.LightCube, timeout=10)
        #cube = robot.world.wait_for_observed_light_cube(timeout=10)
    except asyncio.TimeoutError:
        print('Not enough cubes found')
    finally:
        look_around.stop()
    #robot.go_to_pose(pose_z_angle(cube.pose.position.x - 60, cube.pose.position.y, 0, radians(cube.pose.rotation.angle_z.radians))).wait_for_completed()
    if are_attached(cubes[0], cubes[1], cube_len):
        x1 = cubes[0].pose.position.x
        y1 = cubes[0].pose.position.y
        x2 = cubes[1].pose.position.x
        y2 = cubes[1].pose.position.y
        heading = cubes[0].pose.rotation.angle_z.radians

        custom_obj = CustomObject(((x1+ x2) / 2, (y1 + y2) / 2, heading), 2 * cube_len, cube_len)

        #robot.go_to_pose(pose_z_angle(custom_obj.pose[0], custom_obj.pose[1], 0, radians(custom_obj.pose[2]))).wait_for_completed()

        print(custom_obj)
        visualize()

def handle_viz_input(input):
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        rospy.loginfo(input.marker_name + ' was clicked.')
    else:
        rospy.loginfo('Cannot handle this InteractiveMarker event')

def visualize():
    server = InteractiveMarkerServer("simple_marker")
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = "my_marker"
    int_marker.description = "Simple Click control"
    int_marker.pose.position.x = 1
    int_marker.pose.orientation.w = 1

    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.pose.orientation.w = 1
    box_marker.scale.x = 0.45
    box_marker.scale.y = 0.45
    box_marker.scale.z = 0.45
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    button_control = InteractiveMarkerControl()
    button_control.interaction_mode = InteractiveMarkerControl.BUTTON
    button_control.always_visible = True
    button_control.markers.append(box_marker)
    int_marker.controls.append(button_control)

    server.insert(int_marker, handle_viz_input)
    server.applyChanges()
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('edge_detection')
    visualize()
    try:
        cozmo.run_program(cozmo_run)
    except rospy.ROSInterruptException:
        pass
