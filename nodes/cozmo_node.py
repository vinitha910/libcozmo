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
from cozmo.util import distance_mm

class CozmoROSNode(object):
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
        self.forklift_publisher = rospy.Publisher('forklift_pose', ForkLift, queue_size=1)
        self.headpos_publisher = rospy.Publisher('head_pose', Float64, queue_size=1)
        self.pose_publisher = rospy.Publisher('robot_pose', Pose, queue_size=1)
        self.camera_publisher = rospy.Publisher('camera', Image, queue_size=1)
        self.imu_publisher = rospy.Publisher('imu', Imu, queue_size=1)

    def create_node(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.publish_forklift_pose()
            self.publish_head_pose()
            self.publish_robot_pose()
            self.publish_image()
            self.publish_imu()
            rate.sleep()

    def publish_forklift_pose(self):
        """
        Publisher for forklift, sends a message containing
            height of the forklift above the ground, in millimeters,
            ratio from 0 to 1 of how high the forklift is raised, 0 at the bottom, 1 at the top
            angle of the lift relative to the ground, in radians
        """

        fl = ForkLift()
        fl.height_mm = self.cozmo.lift_height.distance_mm
        fl.ratio = self.cozmo.lift_ratio
        fl.angle_rad = self.cozmo.lift_angle.radians
        self.forklift_publisher.publish(fl)

    def publish_head_pose(self):
        """
        Publisher for head position, sends a message containing
            angle of the head, in radians
        """

        self.headpos_publisher.publish(self.cozmo.head_angle.radians)

    def publish_robot_pose(self):
        """
        Publisher for pose, sends a message containing
            Point: x, y, z positions representing the position of the robot, in millimeters
            Quaternion: x, y, z, w values representing the orientation of the robot
        """
        
        pose = Pose()
        point = Point()
        quat = Quaternion()

        point.x = self.cozmo.pose.position.x
        point.y = self.cozmo.pose.position.y
        point.z = self.cozmo.pose.position.z

        quat.x = self.cozmo.pose.rotation.q0
        quat.y = self.cozmo.pose.rotation.q1
        quat.z = self.cozmo.pose.rotation.q2
        quat.w = self.cozmo.pose.rotation.q3
        pose.position = point
        pose.orientation = quat

        self.pose_publisher.publish(pose)

    def publish_image(self):
        """
        Publisher for camera image, sends a message containing
            320 x 240 greyscale image in the form of a bytes object
            as well as other miscellaneous information about the image
        """

        self.cozmo.camera.image_stream_enabled = True
        camera_image = self.cozmo.world.latest_image
        if camera_image is not None:
            img = camera_image.raw_image.convert('L')
            ros_img = Image()
            ros_img.header.frame_id = 'cozmo_camera'
            image_time = camera_image.image_recv_time
            ros_img.header.stamp = rospy.Time.from_sec(image_time)
            ros_img.height = img.size[1]
            ros_img.width = img.size[0]
            ros_img.encoding = 'mono8'
            ros_img.step = ros_img.width
            ros_img.data = img.tobytes()
            
            self.camera_publisher.publish(ros_img)

    def publish_imu(self):
        """
        Publisher for inertia data, sends a message containing
            orientation, in quaternion
            angular_velocity, in radians/s
            linear_acceleration, in mm/s^2
        """

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

def cozmo_run(robot: cozmo.robot):
    node = CozmoROSNode(robot)
    node.create_node()

if __name__ == '__main__':
    rospy.init_node('cozmo', anonymous=True)
    try:
        cozmo.run_program(cozmo_run)
    except rospy.ROSInterruptException:
        pass
