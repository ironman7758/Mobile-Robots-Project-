#!/usr/bin/env python3
import os, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import torch, torch.nn.functional as F
import cv2, numpy as np
from collections import deque
from ament_index_python.packages import get_package_share_directory
from digit_recognition.model import CNNModel


class DigitRecognitionNode(Node):
    def __init__(self):
        super().__init__('digit_recognition_node')
        self.bridge = CvBridge()

        # ─────────────────── model ───────────────────
        share = get_package_share_directory('digit_recognition')
        model_path = os.path.join(share, 'model', 'mnist_cnn_model.pth')
        self.get_logger().info(f"Loading CNN weights from: {model_path}")
        self.model = CNNModel()
        self.model.load_state_dict(torch.load(model_path, map_location='cpu'))
        self.model.eval()

        # ────────────────── publishers ──────────────────
        self.pub   = self.create_publisher(String, 'digit_classification_result', 10)
        self.annot = self.create_publisher(Image,  'digit_annotated',            10)
        self.paper = self.create_publisher(Image,  'digit_paper_mask',           10)
        self.mask  = self.create_publisher(Image,  'digit_thresh',               10)
        self.roi   = self.create_publisher(Image,  'digit_debug_roi',            1)

        # ─────────────── storage & parameters ───────────────
        self.depth_image = None
        self.fx = self.fy = self.cx = self.cy = None

        self.conf_threshold = 0.94     # only speak when ≥98 % sure
        self.min_interval   = 5        # seconds between messages
        self.last_pub_time  = time.time() - self.min_interval

        # rolling window for “best-of” vote
        self.window_len = 30           # ≈1 s @30 Hz
        self.window     = deque(maxlen=self.window_len)

        # ────────────────── subscriptions ──────────────────
        self.create_subscription(CameraInfo, '/camera/rgb/camera_info',
                                 self.info_callback, 10)
        self.create_subscription(Image, '/camera/depth/image_raw',
                                 self.depth_callback, 10)
        self.create_subscription(CompressedImage,
                                 '/camera/rgb/image_raw/compressedDepth',
                                 self.compressed_depth_callback, 10)
        self.create_subscription(CompressedImage,
                                 '/camera/stereo/image_raw/compressedDepth',
                                 self.compressed_depth_callback, 10)
        self.create_subscription(Image, 'camera/image_raw', self.cb, 10)

        self.get_logger().info('DigitRecognitionNode started.')

    # ───────────────────── callbacks ─────────────────────
    def info_callback(self, msg: CameraInfo):
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]

    def depth_callback(self, msg: Image):
        self.depth_image = CvBridge().imgmsg_to_cv2(msg, 'passthrough')

    def compressed_depth_callback(self, msg: CompressedImage):
        try:
            self.depth_image = cv2.imdecode(
                np.frombuffer(msg.data, np.uint8), cv2.IMREAD_UNCHANGED)
        except Exception as e:
            self.get_logger().warn(f"Failed to decode compressedDepth: {e}")

    # main image callback
    def cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        H, W  = gray.shape

        # 1) ───────── white-paper mask ─────────
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_p = cv2.inRange(hsv, (0, 0, 80), (180, 30, 255))
        mask_p = cv2.morphologyEx(mask_p, cv2.MORPH_OPEN,
                                  cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10)))  #33
        mask_p = cv2.morphologyEx(mask_p, cv2.MORPH_CLOSE,
                                  cv2.getStructuringElement(cv2.MORPH_RECT, (20, 20)))  #1111
        mask_p = cv2.dilate(mask_p,
                            cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1)), 1)  #1010
        self.paper.publish(self.bridge.cv2_to_imgmsg(mask_p, 'mono8'))

        cnts, _ = cv2.findContours(mask_p, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        papers = [(x, y, w, h) for c in cnts
                  for x, y, w, h in [cv2.boundingRect(c)]
                  if 8000 < w*h < 0.6*H*W and 0.6 < h/(w+1e-6) < 1.8]
        if not papers:
            return
        x, y, w, h = max(papers, key=lambda r: r[2]*r[3])
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)

        # 2) ───────── adaptive threshold inside paper ─────────
        pg = gray[y:y+h, x:x+w]
        mask_d = cv2.adaptiveThreshold(pg, 255,
                                       cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY_INV, 15, 5)
        mask_d = cv2.morphologyEx(mask_d, cv2.MORPH_CLOSE,
                                  cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))

        # keep only the largest connected component (digit)
        num, lbl, stats, _ = cv2.connectedComponentsWithStats(mask_d, 8)
        if num <= 1:
            return
        largest = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
        mask_d = np.where(lbl == largest, 255, 0).astype(np.uint8)
        self.mask.publish(self.bridge.cv2_to_imgmsg(mask_d, 'mono8'))

        # 3) ───────── tight bounding box ─────────
        coords = np.column_stack(np.where(mask_d > 0))
        if coords.size == 0:
            return
        dy, dx, dh, dw = cv2.boundingRect(coords)
        pad = 8
        x2, y2 = max(dx-pad, 0),               max(dy-pad, 0)
        x3, y3 = min(dx+dw+pad, pg.shape[1]),  min(dy+dh+pad, pg.shape[0])
        cv2.rectangle(frame, (x+x2, y+y2), (x+x3, y+y3), (0, 255, 0), 2)

        # 4) ───────── 28×28 ROI for CNN ─────────
        roi = mask_d[y2:y3, x2:x3]
        S   = max(x3-x2, y3-y2)
        bx  = (S - (x3-x2)) // 2
        by  = (S - (y3-y2)) // 2
        sq  = cv2.copyMakeBorder(roi, by, S - (y3-y2) - by,
                                 bx, S - (x3-x2) - bx,
                                 cv2.BORDER_CONSTANT, 0)
        img28 = cv2.resize(sq, (28, 28)).astype(np.float32) / 255.0
        img28 = (img28 - 0.5) / 0.5
        self.roi.publish(self.bridge.cv2_to_imgmsg(
            (((img28*0.5)+0.5)*255).astype(np.uint8), 'mono8'))
        
        
        

        # 5) ───────── inference ─────────
        t = torch.from_numpy(img28).unsqueeze(0).unsqueeze(0)
        with torch.no_grad():
            probs = F.softmax(self.model(t), dim=1)[0].cpu().numpy()
        pred_digit = int(np.argmax(probs))
        confidence = float(np.max(probs))

        # 6) ───────── window vote ─────────
        self.window.append((pred_digit, confidence))
        if len(self.window) == self.window_len:
            by_digit = {}
            for d, c in self.window:
                by_digit.setdefault(d, []).append(c)
            pred_smoothed, conf_smoothed = max(
                ((d, sum(cs)/len(cs)) for d, cs in by_digit.items()),
                key=lambda t: t[1])
        else:
            pred_smoothed, conf_smoothed = pred_digit, confidence

        # 7) ───────── locate in 3-D (optional) ─────────
        u = x + x2 + (x3 - x2)//2
        v = y + y2 + (y3 - y2)//2
        if (self.depth_image is not None
                and None not in (self.fx, self.fy, self.cx, self.cy)):
            Z = float(self.depth_image[v, u]) * 0.001   # mm → m
            X = (u - self.cx) * Z / self.fx
            Y = (v - self.cy) * Z / self.fy
            loc_str = f"x={X:.2f},y={Y:.2f},z={Z:.2f}"
        else:
            loc_str = f"u={u},v={v}"

        # ───────── annotate & publish ─────────
        cv2.putText(frame, str(pred_smoothed), (x, max(0, y-10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        self.annot.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))

        now = time.time()
        if (conf_smoothed >= self.conf_threshold
                and now - self.last_pub_time >= self.min_interval):
            self.last_pub_time = now
            msg_txt = f"digit: {pred_smoothed}, conf: {conf_smoothed:.3f}, location: {loc_str}"
            self.get_logger().info(msg_txt)
            self.pub.publish(String(data=msg_txt))

    # ─────────────────── cleanup ───────────────────
    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DigitRecognitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
