import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloNode(Node):
    def _init_(self):
        super()._init_('yolo_node')
        self.subscription = self.create_subscription(
            Image,
            '/downward_camera/image_raw',
            self.listener_callback,
            10)
        self.br = CvBridge()
        self.model = YOLO('/home/thomas/Downloads/best.pt')  # Replace with your YOLO model path

    def listener_callback(self, data):
        # Convert ROS Image to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)

        # YOLO Detection
        results = self.model(current_frame)

        # Draw bounding boxes
        for result in results:
            cv2.rectangle(current_frame, (result['x1'], result['y1']), (result['x2'], result['y2']), (0, 255, 0), 2)

        # Show image
        cv2.imshow("YOLO Detection", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if _name_ == '_main_':
    main()
