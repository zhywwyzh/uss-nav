#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class USBImageProcessor:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('usb_camera_grayscale_publisher', anonymous=True)
        
        # 创建cv_bridge
        self.bridge = CvBridge()
        
        # 创建图像发布者 - 发布灰度图像
        self.gray_pub = rospy.Publisher('/camera/grayscale_image', Image, queue_size=10)
        
        # 初始化摄像头
        self.cap = cv2.VideoCapture(0)  # 0表示默认摄像头:cite[1]
        
        # 设置摄像头分辨率（可选）:cite[2]
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # 检查摄像头是否成功打开
        if not self.cap.isOpened():
            rospy.logerr("无法打开摄像头")
            return
            
        rospy.loginfo("USB摄像头节点已启动")

    def process_and_publish(self):
        rate = rospy.Rate(30)  # 30Hz
        
        while not rospy.is_shutdown():
            # 读取摄像头帧:cite[1]
            ret, frame = self.cap.read()
            
            if not ret:
                rospy.logwarn("无法从摄像头读取帧")
                continue
            
            try:
                # 将图像转换为灰度图:cite[1]
                gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                
                # 将OpenCV图像转换为ROS图像消息
                gray_ros_msg = self.bridge.cv2_to_imgmsg(gray_frame, "mono8")
                
                # 添加时间戳
                gray_ros_msg.header.stamp = rospy.Time.now()
                gray_ros_msg.header.frame_id = "camera_frame"
                
                # 发布灰度图像
                self.gray_pub.publish(gray_ros_msg)
                
                # 可选：显示图像（调试用）
                #cv2.imshow('Original', frame)
               # cv2.imshow('Grayscale', gray_frame)
                
                # 按'q'退出
                #if cv2.waitKey(1) & 0xFF == ord('q'):
                    #break
                    
            except CvBridgeError as e:
                rospy.logerr("CV桥接错误: %s", str(e))
            
            rate.sleep()
        
        # 释放资源
        self.cap.release()
        cv2.destroyAllWindows()

def main():
    try:
        processor = USBImageProcessor()
        processor.process_and_publish()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("节点错误: %s", str(e))

if __name__ == '__main__':
    main()