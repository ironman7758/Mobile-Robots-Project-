#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg    import String
from cv_bridge       import CvBridge
import torch, torch.nn.functional as F
import cv2
import numpy as np
from collections import deque
from ament_index_python.packages import get_package_share_directory
from digit_recognition.model import CNNModel

class DigitRecognitionNode(Node):
    def __init__(self):
        super().__init__('digit_recognition_node')
        self.bridge = CvBridge()

        # Load weights
        share = get_package_share_directory('digit_recognition')
        model_path = os.path.join(share, 'model', 'mnist_cnn_model.pth')
        self.get_logger().info(f"Loading CNN weights from: {model_path}")
        self.model = CNNModel()
        self.model.load_state_dict(torch.load(model_path, map_location='cpu'))
        self.model.eval()

        # Publishers
        self.pub   = self.create_publisher(String, 'digit_classification_result', 10)
        self.annot = self.create_publisher(Image,  'digit_annotated',               10)
        self.paper = self.create_publisher(Image,  'digit_paper_mask',              10)
        self.mask  = self.create_publisher(Image,  'digit_thresh',                  10)
        self.roi   = self.create_publisher(Image,  'digit_debug_roi',               1)

        self.history = deque(maxlen=5)
        self.sub = self.create_subscription(Image, 'camera/image_raw', self.cb, 10)
        self.get_logger().info('DigitRecognitionNode started.')

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        H, W  = gray.shape

        # 1) Find paper in HSV
        hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_p = cv2.inRange(hsv, (0,0,200), (180,50,255))
        mask_p = cv2.morphologyEx(mask_p, cv2.MORPH_OPEN, 
                                   cv2.getStructuringElement(cv2.MORPH_RECT,(3,3)))
        mask_p = cv2.morphologyEx(mask_p, cv2.MORPH_CLOSE,
                                   cv2.getStructuringElement(cv2.MORPH_RECT,(11,11)))
        self.paper.publish(self.bridge.cv2_to_imgmsg(mask_p, 'mono8'))

        cnts,_ = cv2.findContours(mask_p, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        papers = [(x,y,w,h) for c in cnts
                  for x,y,w,h in [cv2.boundingRect(c)]
                  if 5000 < w*h < 0.6*H*W and 0.3 < w/float(h+1e-6) < 3.0]
        if not papers: return
        x,y,w,h = max(papers, key=lambda r:r[2]*r[3])
        cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)

        # 2) Adaptive threshold *inside* paper
        pg = gray[y:y+h, x:x+w]
        # finer blockSize/ C if lighting is rough
        mask_d = cv2.adaptiveThreshold(pg, 255,
                                       cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY_INV,
                                       blockSize=15,
                                       C=5)
        # cleanup
        mask_d = cv2.morphologyEx(mask_d, cv2.MORPH_CLOSE,
                                  cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(4,4)))
        mask_d = cv2.morphologyEx(mask_d, cv2.MORPH_OPEN,
                                  cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2,2)))
        self.mask.publish(self.bridge.cv2_to_imgmsg(mask_d, 'mono8'))

        # 3) Pick the *digit* contour, not the paper
        cnts,_ = cv2.findContours(mask_d, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts: return
        paper_area = mask_d.shape[0]*mask_d.shape[1]
        # sort desc, skip any too big (paper) or too small (noise)
        digit_c = None
        for c in sorted(cnts, key=cv2.contourArea, reverse=True):
            A = cv2.contourArea(c)
            if 200 < A < 0.5*paper_area:
                digit_c = c
                break
        if digit_c is None: return

        dx,dy,dw,dh = cv2.boundingRect(digit_c)
        pad = 5
        x2 = max(dx-pad, 0);  y2 = max(dy-pad, 0)
        x3 = min(dx+dw+pad, mask_d.shape[1])
        y3 = min(dy+dh+pad, mask_d.shape[0])
        cv2.rectangle(frame, (x+x2, y+y2), (x+x3,y+y3), (0,255,0),2)

        # 4) Build 28×28 ROI
        roi = mask_d[y2:y3, x2:x3]
        S   = max(x3-x2, y3-y2)
        bx  = (S - (x3-x2))//2;  by = (S - (y3-y2))//2
        sq  = cv2.copyMakeBorder(roi, by, S-(y3-y2)-by,
                                 bx, S-(x3-x2)-bx,
                                 cv2.BORDER_CONSTANT,0)
        img28 = cv2.resize(sq, (28,28)).astype(np.float32)/255.0
        img28 = (img28 - 0.5)/0.5  # [-1,1]

        # publish debug ROI
        dbg = ((img28*0.5)+0.5)*255
        dbg = dbg.astype(np.uint8)
        self.roi.publish(self.bridge.cv2_to_imgmsg(dbg,'mono8'))

        # 5) CNN inference + smoothing
        t   = torch.from_numpy(img28).unsqueeze(0).unsqueeze(0)
        out = self.model(t)
        probs = F.softmax(out,1)[0].detach().cpu().numpy()
        self.get_logger().info(f"Confidences: {probs.round(3).tolist()}")
        p = int(np.argmax(probs))
        self.history.append(p)
        pred = max(set(self.history), key=self.history.count)

        # 6) Annotate & publish
        cv2.putText(frame, str(pred), (x, max(0,y-10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 2, (0,255,0),3)
        self.annot.publish(self.bridge.cv2_to_imgmsg(frame,'bgr8'))
        self.pub.publish(String(data=str(pred)))

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
