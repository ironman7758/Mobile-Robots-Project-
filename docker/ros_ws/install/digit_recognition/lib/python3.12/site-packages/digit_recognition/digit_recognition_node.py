#!/usr/bin/env python3
import os, time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
import tf2_ros

from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import torch, torch.nn.functional as F
import cv2, numpy as np
from collections import deque
from ament_index_python.packages import get_package_share_directory
from digit_recognition.model import CNNModel


def gamma_correction(image: np.ndarray, gamma: float = 1.2) -> np.ndarray:
    """
    Apply gamma correction to an 8-bit single-channel image.
    """
    inv_gamma = 1.0 / gamma
    table = np.array([(i / 255.0) ** inv_gamma * 255
                      for i in np.arange(256)]).astype("uint8")
    return cv2.LUT(image, table)


class DigitRecognitionNode(Node):
    def __init__(self):
        super().__init__('digit_recognition_node')
        self.bridge = CvBridge()

        # Load MNIST CNN
        share = get_package_share_directory('digit_recognition')
        model_path = os.path.join(share, 'model', 'mnist_cnn_model.pth')
        self.get_logger().info(f"Loading CNN weights from: {model_path}")
        self.model = CNNModel()
        self.model.load_state_dict(torch.load(model_path, map_location='cpu'))
        self.model.eval()

        # Publishers
        self.pub   = self.create_publisher(String, 'digit_classification_result', 10)
        self.annot = self.create_publisher(Image,  'digit_annotated',            10)
        self.paper = self.create_publisher(Image,  'digit_paper_mask',           10)
        self.mask  = self.create_publisher(Image,  'digit_thresh',               10)
        self.roi   = self.create_publisher(Image,  'digit_debug_roi',            1)

        # State
        self.last_printed_digit = None
        self.window_len = 30  # ~1s @30Hz
        self.window = deque(maxlen=self.window_len)

        # TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.create_subscription(CameraInfo, '/camera/rgb/camera_info', self.info_callback, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.create_subscription(CompressedImage, '/camera/rgb/image_raw/compressedDepth', self.compressed_depth_callback, 10)
        self.create_subscription(CompressedImage, '/camera/stereo/image_raw/compressedDepth', self.compressed_depth_callback, 10)
        self.create_subscription(Image, 'camera/image_raw', self.cb, 10)

        self.get_logger().info('DigitRecognitionNode started.')

    def info_callback(self, msg: CameraInfo):
        # Not used here
        pass

    def depth_callback(self, msg: Image):
        # Not used here
        pass

    def compressed_depth_callback(self, msg: CompressedImage):
        # Not used here
        pass

    def cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        H, W = gray.shape

        # 1) White-paper mask
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_p = cv2.inRange(hsv, (0, 0, 80), (180, 30, 255))
        mask_p = cv2.morphologyEx(mask_p, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (10,10)))
        mask_p = cv2.morphologyEx(mask_p, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (20,20)))
        mask_p = cv2.dilate(mask_p, cv2.getStructuringElement(cv2.MORPH_RECT, (1,1)), 1)
        self.paper.publish(self.bridge.cv2_to_imgmsg(mask_p, 'mono8'))

        cnts, _ = cv2.findContours(mask_p, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        papers = [(x,y,w,h) for c in cnts for x,y,w,h in [cv2.boundingRect(c)] if 8000 < w*h < 0.6*H*W and 0.6 < h/(w+1e-6) < 1.8]
        if not papers:
            return
        x,y,w,h = max(papers, key=lambda r: r[2]*r[3])
        cv2.rectangle(frame, (x,y), (x+w,y+h), (255,0,0), 2)

        # 2) Adaptive threshold inside paper
        pg = gray[y:y+h, x:x+w]
        mask_d = cv2.adaptiveThreshold(pg,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,11,2)
        mask_d = cv2.morphologyEx(mask_d, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3)))
        num, lbl, stats, _ = cv2.connectedComponentsWithStats(mask_d, 8)
        if num <= 1:
            return
        largest = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
        mask_d = np.where(lbl==largest,255,0).astype(np.uint8)
        self.mask.publish(self.bridge.cv2_to_imgmsg(mask_d, 'mono8'))

        # 3) Tight bounding box
        coords = np.column_stack(np.where(mask_d>0))
        if coords.size == 0:
            return
        dy,dx,dh,dw = cv2.boundingRect(coords)
        pad=8
        x2,y2 = max(dx-pad,0), max(dy-pad,0)
        x3,y3 = min(dx+dw+pad,pg.shape[1]), min(dy+dh+pad,pg.shape[0])
        cv2.rectangle(frame,(x+x2,y+y2),(x+x3,y+y3),(0,255,0),2)

        # 4) Preprocess ROI: gamma + Otsu + pad & resize
        roi = mask_d[y2:y3, x2:x3]
        roi_gamma = gamma_correction(roi, gamma=20)
        roi_gamma = cv2.convertScaleAbs(roi_gamma, alpha=1.0, beta=15)
        _, roi_otsu = cv2.threshold(roi_gamma,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        S = max(roi_otsu.shape)
        bx = (S - roi_otsu.shape[1])//2
        by = (S - roi_otsu.shape[0])//2
        sq = cv2.copyMakeBorder(roi_otsu, by, S-roi_otsu.shape[0]-by, bx, S-roi_otsu.shape[1]-bx, cv2.BORDER_CONSTANT,0)
        img20 = cv2.resize(sq,(20,20),interpolation=cv2.INTER_AREA).astype(np.float32)/255.0
        img28 = cv2.copyMakeBorder(img20,4,4,4,4,cv2.BORDER_CONSTANT,value=0).astype(np.float32)
        img28 = (img28-0.5)/0.5
        img28 = cv2.GaussianBlur(img28,(3,3),0)
        #img28 = cv2.threshold(img28,127,255,cv2.THRESH_BINARY)
        self.roi.publish(self.bridge.cv2_to_imgmsg(((img28*0.5+0.5)*255).astype(np.uint8),'mono8'))

        #self.roi.publish(self.bridge.cv2_to_imgmsg(((img28)).astype(np.uint8),'mono8'))
        
        # 5) Inference
        t = torch.from_numpy(img28).unsqueeze(0).unsqueeze(0)
        with torch.no_grad():
            probs = F.softmax(self.model(t),dim=1)[0].cpu().numpy()
        pred_digit = int(np.argmax(probs))
        confidence = float(np.max(probs))

        # 6) Smooth vote
        self.window.append((pred_digit,confidence))
        if len(self.window)==self.window_len:
            by_digit={}
            for d,c in self.window:
                by_digit.setdefault(d,[]).append(c)
            pred_smoothed, _ = max(((d,sum(cs)/len(cs)) for d,cs in by_digit.items()), key=lambda t:t[1])
        else:
            pred_smoothed = pred_digit

        # 7) Annotate & publish
        cv2.putText(frame, str(pred_smoothed), (x, max(0,y-10)), cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),2)
        self.annot.publish(self.bridge.cv2_to_imgmsg(frame,'bgr8'))

        # 8) On new digit, lookup TF and publish
        if pred_smoothed != self.last_printed_digit:
            self.last_printed_digit = pred_smoothed
            x_pos = y_pos = None
            ros_now = Time()
            for parent in ('map','odom'):
                if self.tf_buffer.can_transform(parent,'base_link',ros_now,timeout=Duration(seconds=0.5)):
                    trans = self.tf_buffer.lookup_transform(parent,'base_link',ros_now)
                    x_pos = trans.transform.translation.x
                    y_pos = trans.transform.translation.y
                    break
            if x_pos is None:
                self.get_logger().warning('TF unavailable for map/odom → base_link, defaulting 0')
                x_pos = y_pos = 0.0
            msg_txt = f"digit: {pred_smoothed} at x={x_pos:.2f}, y={y_pos:.2f}"
            self.get_logger().info(msg_txt)
            self.pub.publish(String(data=msg_txt))

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DigitRecognitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
