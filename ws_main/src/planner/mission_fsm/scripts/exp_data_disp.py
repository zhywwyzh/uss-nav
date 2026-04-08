#!/usr/bin/env python3
import rospy

from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

br = tf.TransformBroadcaster()
path_pub_ = rospy.Publisher('/drone_path', Path, queue_size = 10)
path = Path()
path.header.frame_id = "world"


def odom_callback(odom_msg):
    position = (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z)
    orientation = (odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)
    # orientation = (0, 0, 0, 1)
    (roll, pitch, yaw) = euler_from_quaternion(orientation)
    new_orientation = quaternion_from_euler(0, 0, yaw)
    br.sendTransform(position, new_orientation, rospy.Time.now(), "robot_frame", "world")
    
    if len(path.poses) > 0:
        if path.poses[-1].header.stamp.to_sec() + 0.1 < odom_msg.header.stamp.to_sec():
            pose_stamped = PoseStamped()
            pose_stamped.header = odom_msg.header
            pose_stamped.pose = odom_msg.pose.pose
            path.poses.append(pose_stamped)
            path_pub_.publish(path)
    else:
        pose_stamped = PoseStamped()
        pose_stamped.header = odom_msg.header
        pose_stamped.pose = odom_msg.pose.pose
        path.poses.append(pose_stamped)
    

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