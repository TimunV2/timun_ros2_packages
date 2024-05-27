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

        self.publisher_ = self.create_publisher(Image, f"{sys.argv[2]}", 10)
        # self.publisher2_ = self.create_publisher(Image, "/camera_bottom", 10)
        self.cv_bridge = CvBridge()

        # video_path = "/home/hasan/Videos/dataset1.mp4" #for using video
        # self.cap = cv2.VideoCapture(video_path)
        self.cap = cv2.VideoCapture(sys.argv[1]) #for using camera 0 for front, 1 for bottom
        # self.cap2 = cv2.VideoCapture("/dev/video0") #for using camera 0 for front, 1 for bottom
        # self.cap = cv2.VideoCapture(0) #for using camera 0 for front, 1 for bottom

        # if not self.cap.isOpened():
        #     self.get_logger().error("Failed to open the camera")
        #     return

        self.timer_ = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        # ret2, frame2 = self.cap2.read()
        if ret:
            try:
                # frame = cv2.resize(frame, (640, 384))
                msg = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
                self.publisher_.publish(msg)
                # self.get_logger().info(f"Publishing1")
            except Exception as e:
                self.get_logger().error(f"Error publishing image: {str(e)}")
        # if ret2:
        #     try:
        #         # frame = cv2.resize(frame, (640, 384))
        #         msg = self.cv_bridge.cv2_to_imgmsg(frame2, "bgr8")
        #         self.publisher2_.publish(msg)
        #         self.get_logger().info(f"Publishing2")
        #     except Exception as e:
        #         self.get_logger().error(f"Error publishing image: {str(e)}")
        
def main(args=None):
    rclpy.init(args=args)
    node=Camera_Publisher_Node()
    rclpy.spin(node)
    rclpy.shutdown()