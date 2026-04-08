#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class ImageSubscriber:
    def __init__(self):
        rospy.init_node('compressed_image_subscriber', anonymous=True)
        self.image_sub = rospy.Subscriber("/camera/depth/image/compressed", CompressedImage, self.callback)
        self.current_image = None
        self.frame = 0

    def callback(self, msg):
        # Convert compressed image to numpy array
        np_arr = np.frombuffer(msg.data, np.uint8)
        # Decode to OpenCV image
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
        self.current_image = image_np

    def display_image(self):
        cv2.namedWindow('Image', cv2.WINDOW_NORMAL)
        cv2.setMouseCallback('Image', self.mouse_callback)

        while not rospy.is_shutdown():
            if self.current_image is not None:
                cv2.imshow('Image', self.current_image)
                key = cv2.waitKey(0) & 0xFF  # Wait for a key press indefinitely
                if key == ord('a'):  # Press 'a' to get the next image
                    print(f"Frame {self.frame} displayed.")
                    self.frame += 1
                    self.current_image = None  # Reset to wait for the next image
                elif key == 27:  # Press 'ESC' to exit
                    break

        cv2.destroyAllWindows()

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.current_image is not None:
            # Get the pixel value at the clicked position
            pixel_value = self.current_image[y, x]
            print(f"Pixel at ({x}, {y}): {pixel_value}")

if __name__ == '__main__':
    subscriber = ImageSubscriber()
    subscriber.display_image()
