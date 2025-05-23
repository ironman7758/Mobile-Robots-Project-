
#!/usr/bin/env python3
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
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
      • every 5s on first detection saves a JPG named:
          <YYYYMMDD_HHMMSS>_<labels>_<x>_<y>.jpg
      • logs “Saved <labels> at x=<x>, y=<y> → <path>”
      • takes the translation of 'base_link' in 'odom' via TF lookup
    """
    def __init__(self):
        super().__init__('color_detection_node')
        self.bridge = CvBridge()

        # HSV thresholds
        self.yellow_lower = np.array([20, 100, 100])
        self.yellow_upper = np.array([50, 255, 255])
        self.red_lower1   = np.array([0,   120, 120])   
        self.red_upper1   = np.array([10,  255, 255])

        # Cool-down interval
        self.last_save     = 0.0
        self.save_interval = 5.0  # seconds

        # Output folder (package/detections)
        base_dir = os.path.dirname(__file__)
        self.output_dir = os.path.join(base_dir, 'detections')
        os.makedirs(self.output_dir, exist_ok=True)

        # TF listener
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions & publishers
        self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.annot_pub = self.create_publisher(Image,  'color_annotated',   10)
        self.bbox_pub  = self.create_publisher(String, 'color_detections', 10)

    def image_callback(self, msg: Image):
        # Convert to OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Build color masks
        yellow_mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        red_mask    = cv2.inRange(hsv, self.red_lower1,   self.red_upper1)

        # Clean noise via morphology
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
        for m in (yellow_mask, red_mask):
            cv2.morphologyEx(m, cv2.MORPH_OPEN,  kernel, dst=m)
            cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel, dst=m)

        # Detect contours and annotate
        detections = []
        specs = [(yellow_mask, 'yellow', (0,255,255)), (red_mask, 'red', (0,0,255))]
        for mask, name, color in specs:
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                area = cv2.contourArea(c)
                threshold = 2000 if name=='yellow' else 500
                if area < threshold:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x,y), (x+w,y+h), color, 2)
                cv2.putText(frame, name, (x, y-6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                detections.append(name)

        # On first detection after interval, take snapshot
        now = time.time()
        if detections and (now - self.last_save) > self.save_interval:
            # lookup the latest transform: base_link in odom
            lookup_time = Time()  # zero = latest
            try:
                trans = self.tf_buffer.lookup_transform(
                    'base_link', 'odom', lookup_time,
                    timeout=Duration(seconds=0.5)
                )
                x_pos = trans.transform.translation.x
                y_pos = trans.transform.translation.y
            except Exception as e:
                self.get_logger().warning(f'🛑 TF lookup failed: {e}')
                x_pos = y_pos = 0.0

            # Build filename and save
            ts     = time.strftime('%Y%m%d_%H%M%S')
            label_str = '_'.join(detections)
            fname  = f"{ts}_{label_str}_{x_pos:.2f}_{y_pos:.2f}.jpg"
            path   = os.path.join(self.output_dir, fname)
            cv2.imwrite(path, frame)
            self.get_logger().info(
                f"📸 Saved {label_str} at x={x_pos:.2f}, y={y_pos:.2f} → {path}"
            )
            self.last_save = now

            # Publish detection labels
            out_msg = String()
            out_msg.data = ','.join(detections)
            self.bbox_pub.publish(out_msg)

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

