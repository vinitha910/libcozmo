#!/usr/bin/env python
import rospy
import tf

from visualization_msgs.msg import Marker

def handle_cozmo_pose(msg, tf_name):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
        (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w),
        rospy.Time.now(),
        tf_name,
        'base_link')

if __name__ == '__main__':
    rospy.init_node('cozmo_tf_broadcaster')
    rospy.Subscriber('cozmo_marker', Marker, handle_cozmo_pose, 'cozmo_frame')
    rospy.Subscriber('object_marker', Marker, handle_cozmo_pose, 'object_frame')
    rospy.spin()
