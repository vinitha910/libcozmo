#!/usr/bin/env python3
import rospy

import cozmo
from cozmo.util import angle_z_to_quaternion, pose_z_angle, radians
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
        print(robot.pose)
        visualize(robot, custom_obj)

def handle_viz_input(input):
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        rospy.loginfo(input.marker_name + ' was clicked.')
    else:
        rospy.loginfo('Cannot handle this InteractiveMarker event')

def visualize(cozmo, custom_obj):
    server = InteractiveMarkerServer("simple_marker")
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = "my_marker"
    int_marker.description = "Custom Object"

    cozmo_marker = Marker()
    cozmo_marker.type = Marker.CUBE
    cozmo_marker.pose.position.x = cozmo.pose.position.x / 100.0
    cozmo_marker.pose.position.y = cozmo.pose.position.y / 100.0
    cozmo_marker.pose.position.z = 1 / 100.0
    cozmo_marker.pose.orientation.x = cozmo.pose.rotation.q1
    cozmo_marker.pose.orientation.y = cozmo.pose.rotation.q2
    cozmo_marker.pose.orientation.z = cozmo.pose.rotation.q3
    cozmo_marker.pose.orientation.w = cozmo.pose.rotation.q0
    cozmo_marker.scale.x = custom_obj.width / 100.0
    cozmo_marker.scale.y = custom_obj.width / 100.0
    cozmo_marker.scale.z = custom_obj.width / 100.0
    cozmo_marker.color.r = 0.0
    cozmo_marker.color.g = 0.5
    cozmo_marker.color.b = 0.5
    cozmo_marker.color.a = 1.0

    box_marker = Marker()
    box_marker.type = Marker.CUBE

    box_marker.pose.position.x = custom_obj.pose[0] / 100.0
    box_marker.pose.position.y = custom_obj.pose[1] / 100.0
    box_marker.pose.position.z = 1 / 100.0
    # Add 90 degrees since cozmo front is x-axis
    box_orientation = angle_z_to_quaternion(radians(custom_obj.pose[2] + math.pi / 2))
    print(box_orientation)
    box_marker.pose.orientation.x = box_orientation[1]
    box_marker.pose.orientation.y = box_orientation[2]
    box_marker.pose.orientation.z = box_orientation[3]
    box_marker.pose.orientation.w = box_orientation[0]

    print(box_marker.pose.orientation.x)
    print(box_marker.pose.orientation.y)
    box_marker.scale.x = custom_obj.length / 100.0
    box_marker.scale.y = custom_obj.width / 100.0
    box_marker.scale.z = custom_obj.width / 100.0
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    button_control = InteractiveMarkerControl()
    button_control.interaction_mode = InteractiveMarkerControl.BUTTON
    button_control.always_visible = True
    button_control.markers.append(box_marker)
    button_control.markers.append(cozmo_marker)

    int_marker.controls.append(button_control)

    server.insert(int_marker, handle_viz_input)
    server.applyChanges()
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('edge_detection')
    try:
        cozmo.run_program(cozmo_run)
    except rospy.ROSInterruptException:
        pass

# TODO: using python2, make publisher for box and cozmo and frames for it so that you can see
# movement; also publish arrows after an action, async backward movement + look for cube ; look for cube behavior?
# or need to cycle
