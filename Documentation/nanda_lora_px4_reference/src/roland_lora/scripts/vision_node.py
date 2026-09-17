#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge
import cv2
import apriltag

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.pub = self.create_publisher(TwistStamped, '/vision/vel_cmd', 10)
        self.detector = apriltag.Detector()
        self.get_logger().info("AprilTag Vision Node Started. Looking for tag...")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # 1. Detect AprilTags
            results = self.detector.detect(gray)
            
            cX, cY = -1, -1
            
            if len(results) > 0:
                tag = results[0]
                cX, cY = tag.center
            else:
                # 2. Fallback: YOLO-style Red Box Detection
                hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
                mask1 = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
                mask2 = cv2.inRange(hsv, (160, 100, 100), (180, 255, 255))
                mask = mask1 + mask2
                contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    largest = max(contours, key=cv2.contourArea)
                    if cv2.contourArea(largest) > 50:
                        M = cv2.moments(largest)
                        if M["m00"] != 0:
                            cX = M["m10"] / M["m00"]
                            cY = M["m01"] / M["m00"]

            if cX != -1 and cY != -1:
                # Image center
                img_h, img_w = cv_image.shape[:2]
                center_x = img_w / 2
                center_y = img_h / 2
                
                err_x = center_x - cX
                err_y = center_y - cY
                
                cmd = TwistStamped()
                cmd.header.stamp = self.get_clock().now().to_msg()
                cmd.header.frame_id = "camera_link"
                
                # Corrected camera axes map (FLU body frame)
                cmd.twist.linear.x = err_y * 0.005
                cmd.twist.linear.y = err_x * 0.005
                
                self.pub.publish(cmd)
                self.get_logger().info(f"AprilTag found! Pixels: ({cX:.0f}, {cY:.0f}) -> Cmd: ({cmd.twist.linear.x:.2f}, {cmd.twist.linear.y:.2f})")
            else:
                pass # No tag found

        except Exception as e:
            self.get_logger().error(f"CV Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
