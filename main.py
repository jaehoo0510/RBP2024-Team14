#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header

class DetermineColor(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.callback, 10)
        self.color_pub = self.create_publisher(Header, '/rotate_cmd', 10)
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            # Listen image topic
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            cv2.imshow('Image', image)
            cv2.waitKey(1)

            # Convert to HSV color space
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Define mask for black color (monitor border)
            lower_black = np.array([0, 0, 0])
            upper_black = np.array([180, 255, 50])
            mask_black = cv2.inRange(hsv_image, lower_black, upper_black)

            # Find contours
            contours, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # Find the largest contour assuming it's the monitor
                largest_contour = max(contours, key=cv2.contourArea)

                if cv2.contourArea(largest_contour) > 2000:  # Ensure the contour is large enough
                    # Get bounding box for the largest contour
                    x, y, w, h = cv2.boundingRect(largest_contour)
                   
                    # Crop the region inside the bounding box
                    monitor_roi = hsv_image[y:y+h, x:x+w]

                    # Define color ranges
                    lower_red1 = np.array([0, 50, 50])
                    upper_red1 = np.array([10, 255, 255])
                    lower_red2 = np.array([170, 50, 50])
                    upper_red2 = np.array([180, 255, 255])
                    lower_green = np.array([50, 50, 50])
                    upper_green = np.array([70, 255, 255])
                    lower_blue = np.array([110, 50, 50])
                    upper_blue = np.array([130, 255, 255])

                    # Create masks for each color
                    mask_red = cv2.inRange(monitor_roi, lower_red1, upper_red1) | cv2.inRange(monitor_roi, lower_red2, upper_red2)
                    mask_green = cv2.inRange(monitor_roi, lower_green, upper_green)
                    mask_blue = cv2.inRange(monitor_roi, lower_blue, upper_blue)

                    # Count the number of non-zero pixels in each mask
                    red_count = cv2.countNonZero(mask_red)
                    green_count = cv2.countNonZero(mask_green)
                    blue_count = cv2.countNonZero(mask_blue)

                    # Debug prints for counts
                    print(f"Red count: {red_count}")
                    print(f"Green count: {green_count}")
                    print(f"Blue count: {blue_count}")

                    # Determine the predominant color
                    if red_count > green_count and red_count > blue_count:
                        frame_id = '-1'  # CW
                    elif blue_count > red_count and blue_count > green_count:
                        frame_id = '+1'  # CCW
                    else:
                        frame_id = '0'  # STOP

                    # Print the result to the terminal
                    print(f"Detected color frame_id: {frame_id}")

                    # Prepare rotate_cmd msg
                    msg = Header()
                    msg.stamp = data.header.stamp
                    msg.frame_id = frame_id

                    # Publish color_state
                    self.color_pub.publish(msg)

        except CvBridgeError as e:
            self.get_logger().error('Failed to convert image: %s' % e)

if __name__ == '__main__':
    rclpy.init()
    detector = DetermineColor()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

	

