import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Camera_Subscriber_Node(Node):
    def __init__(self):
        super().__init__("camera_subscriber_node")
        self.get_logger().info("camera_subscriber_node has been started")

        self.subscription_ = self.create_subscription(
            Image,
            "camera_front",
            self.image_callback,
            10)
        # self.subscription_.assert_liveliness()

        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Webcam Feed", cv_image)
            cv2.waitKey(1)  # Adjust the delay as needed
        except Exception as e:
            self.get_logger().error(f"Error converting image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = Camera_Subscriber_Node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
