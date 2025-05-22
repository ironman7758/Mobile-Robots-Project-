

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg    import String
from cv_bridge       import CvBridge
import torch, cv2
import numpy as np
import time
from collections import deque
from ament_index_python.packages import get_package_share_directory
from digit_recognition.model import CNNModel

class DigitRecognitionNode(Node):
    def __init__(self):
        super().__init__('digit_recognition_node')
        self.bridge = CvBridge()

        # Load the trained MNIST CNN
        share = get_package_share_directory('digit_recognition')
        self.model = CNNModel()
        self.model.load_state_dict(torch.load(
            f"{share}/model/mnist_cnn_model.pth", map_location='cpu'))
        self.model.eval()

        # Publishers
        self.pub            = self.create_publisher(String, 'digit_classification_result', 10)
        self.annot_pub      = self.create_publisher(Image,  'digit_annotated',               10)
        self.paper_pub      = self.create_publisher(Image,  'digit_paper_mask',              10)
        self.digit_mask_pub = self.create_publisher(Image,  'digit_thresh',                  10)

        # History buffer for smoothing
        self.history = deque(maxlen=5)
        # Timer for terminal print throttle
        self.last_print_time = time.time() - 5.0

        # Subscribe to raw camera feed
        self.sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.get_logger().info('DigitRecognitionNode started.')

    def image_callback(self, msg):
        # 1) Convert ROS Image to OpenCV BGR + grayscale
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        H, W  = gray.shape

        # 2) White‐paper mask (full frame, HSV)
        hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_p = cv2.inRange(hsv,
                             np.array([0,   0, 200]),
                             np.array([180, 50, 255]))   #change to whats in color detection.
        k_o = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        k_c = cv2.getStructuringElement(cv2.MORPH_RECT, (11,11))
        mask_p = cv2.morphologyEx(mask_p, cv2.MORPH_OPEN,  k_o)
        mask_p = cv2.morphologyEx(mask_p, cv2.MORPH_CLOSE, k_c)
        self.paper_pub.publish(self.bridge.cv2_to_imgmsg(mask_p, 'mono8'))

        # 3) Find & filter the paper contour
        cnts,_ = cv2.findContours(mask_p, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        papers = []
        for c in cnts:
            x,y,w,h = cv2.boundingRect(c)
            area    = w*h
            ar      = w/float(h+1e-6)
            if 5000 < area < H*W*0.6 and 0.3 < ar < 3.0:
                papers.append((x,y,w,h))
        if not papers:
            return
        x,y,w,h = max(papers, key=lambda r: r[2]*r[3])

        # 4) Crop and threshold the paper region
        paper_gray = gray[y:y+h, x:x+w]
        blur = cv2.GaussianBlur(paper_gray, (3,3), 0)
        _, mask_d = cv2.threshold(
            blur, 0, 255,
            cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        # 5) Morphological cleaning
        k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        k_open  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2,2))
        mask_d = cv2.morphologyEx(mask_d, cv2.MORPH_CLOSE, k_close)
        mask_d = cv2.morphologyEx(mask_d, cv2.MORPH_OPEN,  k_open)

        # 6) Remove tiny components (<200 px²)
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask_d)
        clean = np.zeros_like(mask_d)
        for i in range(1, num_labels):
            if stats[i, cv2.CC_STAT_AREA] > 200:
                clean[labels == i] = 255
        mask_d = clean
        self.digit_mask_pub.publish(self.bridge.cv2_to_imgmsg(mask_d, 'mono8'))

        # 7) Locate the largest digit contour
        dcnts,_ = cv2.findContours(mask_d, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not dcnts:
            return
        dx,dy,dw,dh = cv2.boundingRect(max(dcnts, key=cv2.contourArea))

        # 8) Extract ROI and normalize as MNIST:
        roi = mask_d[dy:dy+dh, dx:dx+dw]
        # 8a) Resize to fit in 20×20 box (keep aspect ratio)
        scale = 20.0 / max(dw, dh)
        nw, nh = int(dw*scale), int(dh*scale)
        resized = cv2.resize(roi, (nw, nh), interpolation=cv2.INTER_AREA)

        # 8b) Center of mass translation into 28×28
        m = cv2.moments(resized)
        if m['m00'] != 0:
            cx = m['m10']/m['m00']
            cy = m['m01']/m['m00']
        else:
            cx, cy = nw/2, nh/2
        shift_x = int(round(14 - cx))
        shift_y = int(round(14 - cy))
        M = np.array([[1, 0, shift_x], [0, 1, shift_y]], dtype=np.float32)
        norm = cv2.warpAffine(resized, M, (28,28), borderValue=0)

        # 8c) Anti-alias (Gaussian) + binarize
        norm = cv2.GaussianBlur(norm, (3,3), 0)
        _, norm = cv2.threshold(norm, 1, 255, cv2.THRESH_BINARY)

        # 9) Prepare tensor for CNN (normalize to [-1,1])
        img28 = norm.astype(np.float32) / 255.0
        img28 = (img28 - 0.5) / 0.5
        t     = torch.from_numpy(img28).unsqueeze(0).unsqueeze(0)

        # 10) Inference + temporal smoothing
        out  = self.model(t)
        p    = int(out.argmax(1).item())
        self.history.append(p)
        pred = max(set(self.history), key=self.history.count)

        # 11) Annotate & publish with location
        x1, y1 = x + dx, y + dy
        cv2.rectangle(frame, (x1, y1), (x1+28, y1+28), (0,255,0), 1)
        cv2.putText(frame, str(pred), (x1, max(y1-10,0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0,255,0), 3)
        self.annot_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
        self.pub.publish(String(data=f"{pred}@{x1},{y1}"))

        # 12) Throttled terminal print every 5s
        now = time.time()
        if now - self.last_print_time >= 5.0:
            print(f"{pred} @ ({x1},{y1})")
            self.last_print_time = now

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

