import rclpy
from rclpy.node import Node
from brping import Ping1D
from timunv2_interfaces.msg import PingData

class Ping_Node(Node):
    def __init__(self):
        super().__init__("ping_node")
        self.get_logger().info("ping_node has been started")
        self.ping_sensor_pub_ = self.create_publisher(PingData, "/ping_data", 10)


        self.ping = PingData()
        self.ping_port = "/dev/ttyBRPING"
        self.myPing = Ping1D()
        self.myPing.connect_serial(self.ping_port,115200)
        self.ping_status_now = False
        self.ping_status_last = True
        self.ping_attach = True
        if self.myPing.initialize() is False:
            self.get_logger().error(f"Failed to connect Echo Sounder")
            self.ping_attach = False
        else:
            self.get_logger().info(f"Echo Sounder Connected")
        self.timer_ = self.create_timer(0.01, self.timer_callback)
    
    def timer_callback(self):
        try:
            received_ping = self.myPing.get_distance_simple()
            if received_ping:
                self.ping.ranges_scan = float(received_ping["distance"])
                self.ping.confidence = float(received_ping["confidence"])
            self.ping_status_now = True
        except Exception as e:
            if self.read_status_now == False and self.read_status_last == True:
                self.get_logger().error(f"Error reading ping data: {str(e)}")
        self.ping_status_last = self.ping_status_now
        self.ping_sensor_pub_.publish(self.ping)


def main(args=None):
    rclpy.init(args=args)
    node=Ping_Node()
    rclpy.spin(node)
    rclpy.shutdown()