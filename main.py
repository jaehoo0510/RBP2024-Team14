import rclpy
from rclpy.node import Node
import numpy as np
import cv2

from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError

class DetermineColor(Node):
    def __init__(self):
        super().__init__('determine_color')
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',self.callback,10)
        self.color_pub = self.create_publisher(Header, '/rotate_cmd', 10)
        self.bridge = CvBridge()
        self.count = 0

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            msg = Header()
            msg.stamp = self.get_clock().now().to_msg()
            msg.frame_id = '0'    
           
            R_Min, R_Max = (0, 0, 100), (85, 120, 255)  # Red
            R_Part = cv2.inRange(image, R_Min, R_Max)
            R_Picture = cv2.bitwise_and(image, image, mask=R_Part)
            R_Pixels = cv2.countNonZero(R_Part)
            
            B_Min, B_Max = (80, 30, 30), (115, 255, 255)  # Blue
            B_Part = cv2.inRange(hsv, B_Min, B_Max)
            B_Picture = cv2.bitwise_and(image, image, mask=B_Part)
            B_Pixels = cv2.countNonZero(B_Part)
            
            G_Min, G_Max = (40, 100, 0), (80, 255, 255)  # Green
            G_Part = cv2.inRange(hsv, G_Min, G_Max)
            G_Picture = cv2.bitwise_and(image, image, mask=G_Part)
            G_Pixels = cv2.countNonZero(G_Part)
            
            Y_Min, Y_Max = (20, 100, 100), (40, 255, 255)  # Yellow
            Y_Part = cv2.inRange(hsv, Y_Min, Y_Max)
            Y_Picture = cv2.bitwise_and(image, image, mask=Y_Part)
            Y_Pixels = cv2.countNonZero(Y_Part)
            
            W_Min, W_Max = (200, 200, 200), (255, 255, 255)  # White
            W_Part = cv2.inRange(image, W_Min, W_Max) 
            W_Picture = cv2.bitwise_and(image, image, mask=W_Part)
            W_Pixels = cv2.countNonZero(W_Part)
            

            if ((R_Pixels  > B_Pixels -5 )  and (R_Pixels > Y_Pixels -5) and (R_Pixels > G_Pixels -5) and (R_Pixels > W_Pixels -5 )):
                msg.frame_id = '-1'
                
            elif ((B_Pixels> R_Pixels -5) and  (B_Pixels > Y_Pixels -5) and (B_Pixels > G_Pixels -5) and(B_Pixels  > W_Pixels -5 )):
                msg.frame_id = '+1'
                
            else:
                msg.frame_id = '0'
               
            cv2.waitKey(1)
            self.color_pub.publish(msg)
           
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    detector = DetermineColor()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


