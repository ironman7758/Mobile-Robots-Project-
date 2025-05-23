#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetectionNode(Node):
    """
    A ROS2 node that subscribes to a camera image topic, detects red and yellow
    regions in the image, and publishes masks and an annotated output.
    """
    def __init__(self):
        super().__init__('color_detection_node')
        self.bridge = CvBridge()

        # HSV thresholds for yellow
        self.yellow_lower = np.array([20, 100, 100])
        self.yellow_upper = np.array([30, 255, 255])
        # HSV thresholds for red (two ranges)
        self.red_lower1 = np.array([0, 100, 100])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([160, 100, 100])
        self.red_upper2 = np.array([179, 255, 255])

        # Publishers for raw masks and annotated image
        self.red_pub = self.create_publisher(Image, 'red_mask', 10)
        self.yellow_pub = self.create_publisher(Image, 'yellow_mask', 10)
        self.annot_pub = self.create_publisher(Image, 'color_annotated', 10)

        # Subscribe to the raw camera feed
        self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg: Image):
        # Convert ROS Image to OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create yellow mask
        yellow_mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        yellow_mask = cv2.morphologyEx(
            yellow_mask,
            cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        )

        # Create red mask (combine two HSV ranges)
        mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(mask1, mask2)
        red_mask = cv2.morphologyEx(
            red_mask,
            cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        )

        # Draw bounding boxes on the original frame
        for mask, color in [(yellow_mask, (0, 255, 255)), (red_mask, (0, 0, 255))]:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                if cv2.contourArea(cnt) > 500:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)

        # Publish masks and annotated image
        self.red_pub.publish(self.bridge.cv2_to_imgmsg(red_mask, 'mono8'))
        self.yellow_pub.publish(self.bridge.cv2_to_imgmsg(yellow_mask, 'mono8'))
        self.annot_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
