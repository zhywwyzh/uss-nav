#!/usr/bin/env python3
import rospy

from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3, Quaternion
from tf.transformations import quaternion_from_euler

br = tf.TransformBroadcaster()

def odom_callback(odom_msg):
    position = (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z)
    orientation = (odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)
    br.sendTransform(position, orientation, rospy.Time.now(), "robot_frame", "world")

def main():
    rospy.init_node('follow_robot_view', anonymous=True)
    odom_topic  = rospy.get_param('~odom_topic', '/odom_topic')
    print('frame_tf] | tf-odom -> ', odom_topic)
    odom_sub    = rospy.Subscriber(odom_topic, Odometry, odom_callback)
    rate        = rospy.Rate(60)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()