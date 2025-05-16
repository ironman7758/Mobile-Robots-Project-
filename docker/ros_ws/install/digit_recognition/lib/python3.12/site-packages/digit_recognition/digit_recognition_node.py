#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg    import String
from cv_bridge       import CvBridge
import torch, cv2
from ament_index_python.packages import get_package_share_directory
from digit_recognition.model import CNNModel

class DigitRecognitionNode(Node):
    def __init__(self):
        super().__init__('digit_recognition_node')
        self.bridge = CvBridge()

        share_dir  = get_package_share_directory('digit_recognition')
        model_path = f"{share_dir}/model/mnist_cnn_model.pth"
        self.model = CNNModel()
        self.model.load_state_dict(torch.load(model_path, map_location='cpu'))
        self.model.eval()

        self.pub        = self.create_publisher(String, 'digit_classification_result', 10)
        self.annot_pub  = self.create_publisher(Image,  'digit_annotated',               10)
        self.thresh_pub = self.create_publisher(Image,  'digit_thresh',                  10)

        self.sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.get_logger().info('DigitRecognitionNode started.')

    def image_callback(self, msg):
        # 1) Convert to OpenCV BGR + grayscale
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray   = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        H, W   = gray.shape

        # 2) Restrict to bottom-center 60% of the frame
        roi_y    = int(H * 0.4)
        roi_x1   = int(W * 0.2)
        roi_x2   = int(W * 0.8)
        search_region = gray[roi_y:, roi_x1:roi_x2]

        # 3) Threshold to find white paper
        _, paper_mask = cv2.threshold(search_region, 200, 255, cv2.THRESH_BINARY)
        paper_mask = cv2.morphologyEx(paper_mask, cv2.MORPH_CLOSE,
                                      cv2.getStructuringElement(cv2.MORPH_RECT,(7,7)))
        # Publish for debugging
        self.thresh_pub.publish(self.bridge.cv2_to_imgmsg(paper_mask, 'mono8'))

        # 4) Find paper contours & filter by size + aspect ratio
        cnts, _ = cv2.findContours(paper_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        paper_contours = []
        for cnt in cnts:
            x, y, w, h = cv2.boundingRect(cnt)
            area = w*h
            ar   = w/float(h + 1e-6)
            # Keep only roughly cone-shaped rectangles
            if 5000 < area < (H*W*0.5) and 0.3 < ar < 3.0:
                paper_contours.append((x,y,w,h))
        if not paper_contours:
            return

        # 5) Choose the biggest paper candidate
        x,y,w,h = max(paper_contours, key=lambda b: b[2]*b[3])
        # Map back into full image coords
        x_full = x + roi_x1
        y_full = y + roi_y
        # Draw the paper box
        cv2.rectangle(cv_img, (x_full,y_full), (x_full+w, y_full+h), (255,0,0), 2)

        # 6) Crop only inside that paper box
        paper_gray = gray[y_full:y_full+h, x_full:x_full+w]

        # 7) Threshold inside the paper for the dark digit
        _, digit_mask = cv2.threshold(paper_gray, 128, 255, cv2.THRESH_BINARY_INV)
        self.thresh_pub.publish(self.bridge.cv2_to_imgmsg(digit_mask, 'mono8'))

        # 8) Find digit contour
        dcnts, _ = cv2.findContours(digit_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not dcnts:
            return
        dx, dy, dw, dh = cv2.boundingRect(max(dcnts, key=cv2.contourArea))
        # Draw the digit box
        cv2.rectangle(cv_img, (x_full+dx, y_full+dy),
                             (x_full+dx+dw, y_full+dy+dh),
                             (0,255,0), 2)

        # 9) Prepare the cropped digit for the CNN
        cropped = digit_mask[dy:dy+dh, dx:dx+dw]
        size    = max(dw, dh)
        bx      = (size-dw)//2
        by      = (size-dh)//2
        squared = cv2.copyMakeBorder(cropped, by, size-dh-by, bx, size-dw-bx,
                                     cv2.BORDER_CONSTANT, 0)
        img28   = cv2.resize(squared, (28,28)) / 255.0
        t       = torch.tensor(img28, dtype=torch.float32).unsqueeze(0).unsqueeze(0)

        # 10) Classify & overlay
        out  = self.model(t)
        pred = int(out.argmax(dim=1).item())
        cv2.putText(cv_img, str(pred), (x_full, max(y_full-10,0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0,255,0), 3)
        self.get_logger().info(f"Detected digit: {pred}")

        # 11) Publish results
        self.pub.publish(String(data=str(pred)))
        self.annot_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, 'bgr8'))

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DigitRecognitionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
