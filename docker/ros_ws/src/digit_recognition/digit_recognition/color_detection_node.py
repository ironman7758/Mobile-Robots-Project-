#!/usr/bin/env python3
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
import tf2_ros

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetectionNode(Node):
    """
    A ROS2 node that:
      • subscribes to /camera/image_raw
      • detects red/yellow blobs
      • every 5 s on first detection saves a JPG named:
          <YYYYMMDD_HHMMSS>_<labels>_<x>_<y>.jpg
      • logs “Saved <labels> at x=<x>, y=<y> → <path>”
      • pulls (x,y) from TF map→base_link, with a 0.5 s timeout
    """
    def __init__(self):
        super().__init__('color_detection_node')
        self.bridge = CvBridge()

        # HSV thresholds
        self.yellow_lower = np.array([10, 100, 100])
        self.yellow_upper = np.array([40, 255, 255])
        self.red_lower1   = np.array([0, 100, 100])
        self.red_upper1   = np.array([5, 255, 255])
        self.red_lower2   = np.array([160, 100, 100])
        self.red_upper2   = np.array([179, 255, 255])

        # Cool-down
        self.last_save     = 0.0
        self.save_interval = 5.0  # seconds

        # Output folder
        base_dir = os.path.dirname(__file__)
        self.output_dir = os.path.join(base_dir, 'detections')
        os.makedirs(self.output_dir, exist_ok=True)

        # TF listener
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscription & publishers
        self.create_subscription(Image,  'camera/image_raw',  self.image_callback, 10)
        self.annot_pub = self.create_publisher(Image,  'color_annotated', 10)
        self.bbox_pub  = self.create_publisher(String, 'color_detections',10)

    def image_callback(self, msg: Image):
        # Convert to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Build masks
        yellow_mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        red_mask    = cv2.bitwise_or(
            cv2.inRange(hsv, self.red_lower1, self.red_upper1),
            cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        )

        # Morphology to clean up noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
        for m in (yellow_mask, red_mask):
            cv2.morphologyEx(m, cv2.MORPH_OPEN,  kernel, dst=m)
            cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel, dst=m)

        # Contour detection & annotation
        detections = []
        for mask, name, color in [
            (yellow_mask, 'yellow', (0,255,255)),
            (red_mask,    'red',    (0,0,255))
        ]:
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                area = cv2.contourArea(c)
                thresh = 2000 if name=='yellow' else 500
                if area < thresh:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x,y), (x+w,y+h), color, 2)
                cv2.putText(frame, name, (x, y-6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                detections.append(name)

        # Snapshot logic
        now = time.time()
        if detections and (now - self.last_save) > self.save_interval:
            # Attempt to get current (x,y) from TF
            ros_now = Time()
            if self.tf_buffer.can_transform('map', 'base_link', ros_now,
                                            timeout=Duration(seconds=0.5)):
                trans = self.tf_buffer.lookup_transform('map', 'base_link', ros_now)
                x_pos = trans.transform.translation.x
                y_pos = trans.transform.translation.y
            else:
                self.get_logger().warning('🛑 TF map→base_link not available yet!')
                x_pos = y_pos = 0.0

            # Build filename
            timestamp = time.strftime('%Y%m%d_%H%M%S')
            labels    = "_".join(detections)  # e.g., "red_yellow"
            fname     = f"{timestamp}_{labels}_{x_pos:.2f}_{y_pos:.2f}.jpg"
            path      = os.path.join(self.output_dir, fname)

            # Save & log
            cv2.imwrite(path, frame)
            self.get_logger().info(
                f"📸 Saved {labels} at x={x_pos:.2f}, y={y_pos:.2f} → {path}"
            )
            self.last_save = now

            # Publish detection names
            info = String()
            info.data = ",".join(detections)
            self.bbox_pub.publish(info)

        # Always publish annotated image
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
