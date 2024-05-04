import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

class Camera_Publisher_Node(Node):
    def __init__(self):
        super().__init__("camera_publisher_node")
        self.get_logger().info("camera_publisher_node has been started")
        # self.get_logger().info(f"argument {sys.argv[1]}, {sys.argv[2]}") 
        #Argument: {/dev/video} {/topic_name}

        self.publisher_ = self.create_publisher(Image, f"{sys.argv[2]}", 10)
        self.cv_bridge = CvBridge()

        video_path = "/home/hasan/Videos/dataset1.mp4" #for using video
        # self.cap = cv2.VideoCapture(video_path)
        self.cap = cv2.VideoCapture(int(sys.argv[1]))# for using arguments
        # self.cap = cv2.VideoCapture(0) #for using camera 0 for front, 1 for bottom

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open the camera")
            return

        self.timer_ = self.create_timer(0.03, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            try:
                frame = cv2.resize(frame, (854, 480))
                msg = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
                self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Error publishing image: {str(e)}")
        
def main(args=None):
    rclpy.init(args=args)
    node=Camera_Publisher_Node()
    rclpy.spin(node)
    rclpy.shutdown()