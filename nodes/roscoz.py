#!/usr/bin/env python3

"""
This file implements publishers for various states and sensors of the Cozmo robot.
These include forklift position, head position, robot pose, camera data, and inertia data.
"""
import rospy
from libcozmo.msg import ForkLift
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import Image, Imu

import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps

class CozmoROS(object):
    """
    A Cozmo node class that contains the following topics:
    forklift_position, head_position, pose, camera, imu
    """
    def __init__(self, cozmo):
        """
        Parameters
        ----------
        cozmo : cozmo.robot
            the cozmo SDK robot handle
        """
        
        self.cozmo = cozmo
        self.forklift_publisher = rospy.Publisher('forklift_position', ForkLift, queue_size=1)
        self.headpos_publisher = rospy.Publisher('head_position', Float64, queue_size=1)
        self.pose_publisher = rospy.Publisher('pose', Pose, queue_size=1)
        self.camera_publisher = rospy.Publisher('camera', Image, queue_size=1)
        self.imu_publisher = rospy.Publisher('imu', Imu, queue_size=1)

    def forklift_node(self):
        """
        Publisher for forklift, sends a message containing
            height of the forklift,
            ratio from 0 to 1 of how high the lift is,
            angle of the lift relative to the ground,
        """

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            fl = ForkLift()
            fl.height = self.cozmo.lift_height.distance_mm
            fl.ratio = self.cozmo.lift_ratio
            fl.angle = self.cozmo.lift_angle.radians
            self.forklift_publisher.publish(fl)
            rate.sleep()

    def headpos_node(self):
        """
        Publisher for head position, sends a message containing
            angle of the head
        """

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.headpos_publisher.publish(self.cozmo.head_angle.radians)
            rate.sleep()

    def robotpos_node(self):
        """
        Publisher for pose, sends a message containing
            Point: x, y, z positions representing the position of the robot
            Quaternion: x, y, z, w values representing the orientation of the robot
        """
        # TODO: reset robot's values or set it some specific configuration so that
        #       movement in the robot correspond to those relative locations
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            pose = Pose()
            point = Point()
            quat = Quaternion()

            # point x, y, z values in millimeters
            point.x = self.cozmo.pose.position.x
            point.y = self.cozmo.pose.position.y
            point.z = self.cozmo.pose.position.z

            # Unsure if right corresponding values
            quat.x = self.cozmo.pose.rotation.q0
            quat.y = self.cozmo.pose.rotation.q1
            quat.z = self.cozmo.pose.rotation.q2
            quat.w = self.cozmo.pose.rotation.q3
            pose.position = point
            pose.orientation = quat

            self.pose_publisher.publish(pose)
            rate.sleep()

    def image_node(self):
        """

        Publisher for camera image, sends a message containing
            RGB image in the form of a bytes object (cozmo's camera provides 320 x 240 grayscale images)
            as well as other miscellaneous information about the image
        """

        self.cozmo.camera.image_stream_enabled = True
        while not rospy.is_shutdown():
            camera_image = self.cozmo.world.latest_image
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
                
                self.camera_publisher.publish(ros_img)
                rospy.sleep(5.)

    def imu_node(self):
        """
        Publisher for inertia data, sends a message containing
            orientation
            angular_velocity, in radians/s
            linear_acceleration, in mm/s^2
        """

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            imu = Imu()
            imu.header.stamp = rospy.Time.now()
            imu.header.frame_id = rospy.get_param('~base_frame', 'base_link')
            imu.orientation.w = self.cozmo.pose.rotation.q0
            imu.orientation.x = self.cozmo.pose.rotation.q1
            imu.orientation.y = self.cozmo.pose.rotation.q2
            imu.orientation.z = self.cozmo.pose.rotation.q3
            imu.angular_velocity.x = self.cozmo.gyro.x
            imu.angular_velocity.y = self.cozmo.gyro.y
            imu.angular_velocity.z = self.cozmo.gyro.z
            imu.linear_acceleration.x = self.cozmo.accelerometer.x
            imu.linear_acceleration.y = self.cozmo.accelerometer.y
            imu.linear_acceleration.z = self.cozmo.accelerometer.z

            self.imu_publisher.publish(imu)
            rate.sleep()

def cozmo_run(robot: cozmo.robot):
    node = CozmoROS(robot)
    #node.forklift_node()
    #node.headpos_node()
    #node.robotpos_node()
    #node.image_node()
    node.imu_node()

if __name__ == '__main__':
    rospy.init_node('cozmo', anonymous=True)
    try:
        cozmo.run_program(cozmo_run)
    except rospy.ROSInterruptException:
        pass
