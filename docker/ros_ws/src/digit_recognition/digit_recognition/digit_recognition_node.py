#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import torch
import torch.nn.functional as F
import cv2
import numpy as np
from collections import deque
from ament_index_python.packages import get_package_share_directory
from digit_recognition.model import CNNModel

class DigitRecognitionNode(Node):
    def __init__(self):
        super().__init__('digit_recognition_node')
        self.bridge = CvBridge()

        # Load the trained MNIST CNN
        share = get_package_share_directory('digit_recognition')
        model_path = os.path.join(share, 'model', 'mnist_cnn_model.pth')
        self.get_logger().info(f"Loading CNN weights from: {model_path}")
        self.model = CNNModel()
        self.model.load_state_dict(torch.load(model_path, map_location='cpu'))
        self.model.eval()

        # Publishers
        self.pub = self.create_publisher(String, 'digit_classification_result', 10)
        self.annot = self.create_publisher(Image, 'digit_annotated', 10)
        self.paper = self.create_publisher(Image, 'digit_paper_mask', 10)
        self.mask = self.create_publisher(Image, 'digit_thresh', 10)
        self.roi = self.create_publisher(Image, 'digit_debug_roi', 1)

        # Storage for depth and camera intrinsics
        self.depth_image = None
        self.fx = self.fy = self.cx = self.cy = None
        self.history = deque(maxlen=5)

        # Subscriptions
        # Intrinsics from camera_info
        self.create_subscription(
            CameraInfo, '/camera/rgb/camera_info', self.info_callback, 10
        )
        # Raw depth if available
        self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10
        )
                # Fallback: compressedDepth decoding (RGB and stereo)
        self.create_subscription(
            CompressedImage,
            '/camera/rgb/image_raw/compressedDepth',
            self.compressed_depth_callback,
            10
        )
        self.create_subscription(
            CompressedImage,
            '/camera/stereo/image_raw/compressedDepth',
            self.compressed_depth_callback,
            10
        )
        # Input camera image for digit detection for digit detection
        self.sub = self.create_subscription(
            Image, 'camera/image_raw', self.cb, 10
        )
        self.get_logger().info('DigitRecognitionNode started.')

    def info_callback(self, msg: CameraInfo):
        # store intrinsics
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.get_logger().info(
            f"📐 Got intrinsics: fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}"
        )

    def depth_callback(self, msg: Image):
        # store raw depth image
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough'
        )
        self.get_logger().info(
            f"🌊 Depth frame received: shape={self.depth_image.shape}, "
            f"dtype={self.depth_image.dtype}"
        )

    def compressed_depth_callback(self, msg: CompressedImage):
        # decode compressedDepth into a cv2 image
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            self.depth_image = cv_img
            self.get_logger().info(
                f"🌊 Compressed depth decoded: shape={cv_img.shape}, "
                f"dtype={cv_img.dtype}"
            )
        except Exception as e:
            self.get_logger().warn(f"Failed to decode compressedDepth: {e}")

    def cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        H, W = gray.shape

        # 1) Find paper mask in HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_p = cv2.inRange(hsv, (0,0,200), (180,50,255))
        mask_p = cv2.morphologyEx(
            mask_p, cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
        )
        mask_p = cv2.morphologyEx(
            mask_p, cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_RECT,(11,11))
        )
        self.paper.publish(self.bridge.cv2_to_imgmsg(mask_p, 'mono8'))

        # find paper contour
        cnts,_ = cv2.findContours(
            mask_p, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        papers = [
            (x,y,w,h)
            for c in cnts for x,y,w,h in [cv2.boundingRect(c)]
            if 5000 < w*h < 0.6*H*W and 0.3 < w/(h+1e-6) < 3.0
        ]
        if not papers:
            return
        x,y,w,h = max(papers, key=lambda r: r[2]*r[3])
        cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)

        # 2) Adaptive threshold inside paper
        pg = gray[y:y+h, x:x+w]
        mask_d = cv2.adaptiveThreshold(
            pg,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,blockSize=15,C=5
        )
        mask_d = cv2.morphologyEx(
            mask_d, cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(4,4))
        )
        mask_d = cv2.morphologyEx(
            mask_d, cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2,2))
        )
        self.mask.publish(self.bridge.cv2_to_imgmsg(mask_d, 'mono8'))

        # 3) Find digit contour
        cnts,_ = cv2.findContours(
            mask_d, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not cnts:
            return
        paper_area = mask_d.shape[0] * mask_d.shape[1]
        digit_c = None
        for c in sorted(cnts, key=cv2.contourArea, reverse=True):
            A = cv2.contourArea(c)
            if 200 < A < 0.5*paper_area:
                digit_c = c
                break
        if digit_c is None:
            return
        dx,dy,dw,dh = cv2.boundingRect(digit_c)
        pad = 5
        x2 = max(dx-pad, 0);
        y2 = max(dy-pad, 0)
        x3 = min(dx+dw+pad, mask_d.shape[1])
        y3 = min(dy+dh+pad, mask_d.shape[0])
        cv2.rectangle(frame, (x+x2, y+y2), (x+x3, y+y3), (0,255,0),2)

        # 4) Prepare 28×28 ROI
        roi = mask_d[y2:y3, x2:x3]
        S = max(x3-x2, y3-y2)
        bx = (S - (x3-x2))//2;
        by = (S - (y3-y2))//2
        sq = cv2.copyMakeBorder(
            roi, by, S-(y3-y2)-by, bx, S-(x3-x2)-bx,
            cv2.BORDER_CONSTANT,0
        )
        img28 = cv2.resize(sq, (28,28)).astype(np.float32)/255.0
        img28 = (img28 - 0.5)/0.5
        dbg = ((img28*0.5)+0.5)*255
        dbg = dbg.astype(np.uint8)
        self.roi.publish(self.bridge.cv2_to_imgmsg(dbg,'mono8'))

        # 5) Inference + smoothing
        t = torch.from_numpy(img28).unsqueeze(0).unsqueeze(0)
        out = self.model(t)
        probs = F.softmax(out,1)[0].detach().cpu().numpy()
        p = int(np.argmax(probs))
        self.history.append(p)
        pred = max(set(self.history), key=self.history.count)

        # 6) Compute location
        u = x + x2 + (x3-x2)//2
        v = y + y2 + (y3-y2)//2
        if self.depth_image is not None and None not in (self.fx,self.fy,self.cx,self.cy):
            Z = float(self.depth_image[v,u]) * 0.001
            X = (u - self.cx) * Z / self.fx
            Y = (v - self.cy) * Z / self.fy
            loc = f"x={X:.2f},y={Y:.2f},z={Z:.2f}"
        else:
            loc = f"u={u},v={v}"

        # log and annotate
        self.get_logger().info(f"Detected digit {pred} at {loc}")
        cv2.putText(frame, f"{pred} ({loc})", (x, max(0,y-10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),2)
        self.annot.publish(self.bridge.cv2_to_imgmsg(frame,'bgr8'))
        self.pub.publish(String(data=f"digit: {pred}, location: {loc}"))

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = DigitRecognitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
