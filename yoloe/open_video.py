#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def publish_video():
    # 初始化ROS节点
    rospy.init_node('video_publisher')
    pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
    bridge = CvBridge()
    video_path = rospy.get_param('~video_path', '/mnt/d/Downloads/test_yoloe_video.mp4')
    cap = cv2.VideoCapture(video_path)
    
    if not cap.isOpened():
        rospy.logerr(f"无法打开视频文件: {video_path}")
        return
    
    # 获取视频帧率
    fps = cap.get(cv2.CAP_PROP_FPS)
    rate = rospy.Rate(fps if fps > 0 else 30)  # 默认30FPS
    rospy.loginfo(f"开始发布视频: {video_path}")
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        
        if not ret:
            rospy.loginfo("视频播放完毕")
            break
            
        try:
            img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            img_msg.header.stamp = rospy.Time.now()
            pub.publish(img_msg)
            
        except Exception as e:
            rospy.logerr(f"发布图像时出错: {str(e)}")
            break
            
        rate.sleep()
    
    # 释放资源
    cap.release()
    rospy.loginfo("视频发布节点已关闭")

if __name__ == '__main__':
    try:
        publish_video()
    except rospy.ROSInterruptException:
        pass