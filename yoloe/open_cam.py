import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def camera_publisher():
    # 初始化ROS节点
    rospy.init_node('camera_publisher', anonymous=True)
    
    # 创建图像发布者，发布到/camera/color/image_raw话题
    image_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
    
    # 设置发布频率
    rate = rospy.Rate(30)  # 30Hz
    
    # 创建CvBridge实例，用于OpenCV图像与ROS图像消息的转换
    bridge = CvBridge()
    
    # 打开摄像头（通常笔记本内置摄像头为0）
    cap = cv2.VideoCapture(0)
    
    # 检查摄像头是否成功打开
    if not cap.isOpened():
        rospy.logerr("无法打开摄像头")
        return
    
    rospy.loginfo("摄像头已成功打开，开始发布图像...")
    
    while not rospy.is_shutdown():
        # 读取一帧图像
        ret, frame = cap.read()
        
        if not ret:
            rospy.logerr("无法获取图像帧")
            break
        
        try:
            # 将OpenCV图像转换为ROS图像消息
            img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            
            # 设置图像消息的时间戳
            img_msg.header.stamp = rospy.Time.now()
            
            # 发布图像消息
            image_pub.publish(img_msg)
            
            # 显示发布信息
            rospy.logdebug("已发布一帧图像")
            
        except Exception as e:
            rospy.logerr(f"转换或发布图像时出错: {str(e)}")
        
        # 按照设定的频率休眠
        rate.sleep()
    
    # 释放摄像头资源
    cap.release()
    rospy.loginfo("摄像头已释放，节点关闭")

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass