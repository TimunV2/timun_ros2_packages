import rclpy
from rclpy.node import Node
from brping import Ping1D
from timunv2_interfaces.msg import PingData, SensorData, SetPoint, PipeDetect, JoyUtilities, RecordData
from geometry_msgs.msg import Twist

class Ping_Node(Node):
    def __init__(self):
        super().__init__("Record_node")
        self.get_logger().info("Record_node has been started")
        self.ping_sensor_sub_ = self.create_subscription(PingData, "/ping_data", self.ping_callback, 10)
        self.master_cmd_vel_sub_ = self.create_subscription(Twist, "/master_cmd_vel", self.master_cmd_vel_callback, 10)
        self.joy_cmd_utl_sub_ = self.create_subscription(JoyUtilities, "/joy_cmd_utl", self.joy_cmd_utl_callback, 10)
        self.serial_sensor_data_sub_ = self.create_subscription(SensorData, "/serial_sensor_data", self.serial_sensor_data_callback, 10)
        self.pipeline_cmd_vel_sub_ = self.create_subscription(Twist, "/pipeline_cmd_vel", self.pipeline_cmd_vel_callback, 10)
        self.master_set_point_sub_ = self.create_subscription(SetPoint, "/master_set_point", self.master_set_point_callback, 10)
        self.pipeline_sub_ = self.create_subscription(PipeDetect, "pipeline_value", self.pipeline_detect_callback, 10)
        
        self.record_pub_ = self.create_publisher(RecordData,"/record_data",10)
        self.record_timer_ = self.create_timer(0.01, self.record_timer)
        self.record = RecordData() 
    
    def record_timer(self):
        self.record_pub_.publish(self.record)
    def ping_callback(self,msg):
        self.record.brping = msg
    def master_cmd_vel_callback(self,msg):
        self.record.master_vel = msg
    def joy_cmd_utl_callback(self,msg):
        self.record.joy_utl = msg
    def serial_sensor_data_callback(self,msg):
        self.record.sensor = msg
    def pipeline_cmd_vel_callback(self,msg):
        self.record.pipe_vel = msg
    def master_set_point_callback(self,msg):
        self.record.set_point = msg
    def pipeline_detect_callback(self,msg):
        self.record.pipe = msg

def main(args=None):
    rclpy.init(args=args)
    node=Ping_Node()
    rclpy.spin(node)
    rclpy.shutdown()