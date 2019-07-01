#!/usr/bin/env python3

"""
This file implements publishers for various states and sensors of the Cozmo robot.
These include forklift position, head position, robot pose, camera data, and inertia data.
"""

import rospy
from beginner_tutorials.msg import ForkLift
from std_msgs.msg import Float64
from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion
)
from sensor_msgs.msg import (
    Image,
    Imu
)

import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps

# Publisher for forklift, sends a message containing
# height of the forklift,
# ratio from 0 to 1 of how high the lift is,
# angle of the lift relative to the ground,
def forklift_node(robot: cozmo.robot):
   pub = rospy.Publisher('lift_position', ForkLift, queue_size=1)
   rospy.init_node('forklift_node', anonymous=True)
   rate = rospy.Rate(10) # 10hz
   while not rospy.is_shutdown():
       fl = ForkLift()
       fl.lift_height = robot.lift_height.distance_mm
       fl.lift_ratio = robot.lift_ratio
       fl.lift_angle = robot.lift_angle.radians
       pub.publish(fl)
       rate.sleep()

# Publisher for head position, sends a message containing
# angle of the head
def headpos_node(robot: cozmo.robot):
    pub = rospy.Publisher('head_position', Float64, queue_size=1)
    rospy.init_node('headpos_node', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(robot.head_angle.radians)
        rate.sleep()

# Publisher for pose, sends a message containing
# Point: x, y, z positions representing the position of the robot
# Quaternion: x, y, z, w values representing the orientation of the robot
# TODO: reset robot's values or set it some specific configuration so that
#       movement in the robot correspond to those relative locations
def robotpos_node(robot: cozmo.robot):
    pub = rospy.Publisher('robot_position', Pose, queue_size=1)
    rospy.init_node('robotpos_node', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pose = Pose()
        point = Point()
        quat = Quaternion()

        # point x, y, z values in millimeters
        point.x = robot.pose.position.x
        point.y = robot.pose.position.y
        point.z = robot.pose.position.z

        # Unsure if right corresponding values
        quat.x = robot.pose.rotation.q0
        quat.y = robot.pose.rotation.q1
        quat.z = robot.pose.rotation.q2
        quat.w = robot.pose.rotation.q3
        pose.position = point
        pose.orientation = quat

        pub.publish(pose)
        rate.sleep()

# Publisher for camera image, sends a message containing
# a RGB image in the form of a bytes object
# as well as other miscellaneous information about the image
# I haven't thoroughly tested this publisher yet
def image_node(robot: cozmo.robot):
    pub = rospy.Publisher('camera_image', Image, queue_size=10)
    rospy.init_node('image_node', anonymous=True)
    rate = rospy.Rate(10)
    robot.camera.image_stream_enabled = True
    while not rospy.is_shutdown():
        camera_image = robot.world.latest_image
        if camera_image is not None:
            # There's also 'L' for greyscale, and 'CMYK'
            img = camera_image.raw_image.convert('RGB')
            ros_img = Image()
            ros_img.header.frame_id = 'cozmo_camera'
            image_time = camera_image.image_recv_time
            ros_img.header.stamp = rospy.Time.from_sec(image_time)
            ros_img.height = img.size[1]
            ros_img.width = img.size[0]
            ros_img.encoding = 'rgb8'
            # Unsure about step length - suppose to represent row length in bytes
            ros_img.step = ros_img.width
            ros_img.data = img.tobytes()
            
            pub.publish(ros_img)
            rate.sleep()

# Publisher for inertia data, sends a message containing
# orientation
# angular_velocity, in radians/s
# linear_acceleration, in mm/s^2
def imu_node(robot: cozmo.robot):
    pub = rospy.Publisher('imu', Imu, queue_size = 10)
    rospy.init_node('imu_node', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        imu = Imu()
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = rospy.get_param('~base_frame', 'base_link')
        imu.orientation.w = robot.pose.rotation.q0
        imu.orientation.x = robot.pose.rotation.q1
        imu.orientation.y = robot.pose.rotation.q2
        imu.orientation.z = robot.pose.rotation.q3
        imu.angular_velocity.x = robot.gyro.x
        imu.angular_velocity.y = robot.gyro.y
        imu.angular_velocity.z = robot.gyro.z
        imu.linear_acceleration.x = robot.accelerometer.x
        imu.linear_acceleration.y = robot.accelerometer.y
        imu.linear_acceleration.z = robot.accelerometer.z

        pub.publish(imu)
        rate.sleep()


if __name__ == '__main__':
   try:
       cozmo.run_program(imu_node)

       #robot = connect_to_cozmo_function
       #create_cozmo_node(robot)
   except rospy.ROSInterruptException:
       pass
