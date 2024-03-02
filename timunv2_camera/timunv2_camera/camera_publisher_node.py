import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Camera_Publisher_Node(Node):
    def __init__(self):
        super().__init__("camera_publisher_node")
        self.get_logger().info("camera_publisher_node has been started")

        self.publisher_ = self.create_publisher(Image, "camera_front", 10)
        self.cv_bridge = CvBridge()

        video_path = "/home/hasan/BelajarYOLO/YOLOV8_Pipelinetracking/Pipeline_Youtube.mp4" #for using cideo
        self.cap = cv2.VideoCapture(video_path)
        # self.cap = cv2.VideoCapture(0) #for using camera 0 for front, 1 for bottom

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open the camera")
            return

        self.timer_ = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            try:
                frame = cv2.resize(frame, (640, 384))
                msg = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
                self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Error publishing image: {str(e)}")
        
def main(args=None):
    rclpy.init(args=args)
    node=Camera_Publisher_Node()
    rclpy.spin(node)
    rclpy.shutdown()