import rclpy
from rclpy.node import Node
from brping import Ping1D
from timunv2_interfaces.msg import PingData, SensorData, SetPoint, PipeDetect, JoyUtilities, RecordData, VisdomData
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
        self.visdom_sub_ = self.create_subscription(VisdomData, "/visdom_value",self.visdom_callback,10)

        self.record_pub_ = self.create_publisher(RecordData,"/record_data",10)
        self.record_timer_ = self.create_timer(0.01, self.record_timer)
        self.record = RecordData() 
    
    def record_timer(self):
        self.record_pub_.publish(self.record)
    def visdom_callback(self,msg):
        self.record.vo_x = msg.vo_x
        self.record.vo_y = msg.vo_y
        self.record.vo_z = msg.vo_z
        self.record.count_vo_record = msg.count_vo_record
    def ping_callback(self,msg):
        self.record.ranges_scan = msg.ranges_scan
        self.record.confidence = msg.confidence
    def master_cmd_vel_callback(self,msg):
        self.record.master_linear.x = msg.linear.x
        self.record.master_linear.y = msg.linear.y
        self.record.master_linear.z = msg.linear.z
        self.record.master_angular.x = msg.angular.x
        self.record.master_angular.y = msg.angular.y
        self.record.master_angular.z = msg.angular.z
    def joy_cmd_utl_callback(self,msg):
        self.record.arm_hw = msg.arm_hw
        self.record.arm_sw = msg.arm_sw
        self.record.stabilize = msg.stabilize
        self.record.depthhold = msg.depthhold
        self.record.imu_reset = msg.imu_reset
        self.record.lumen = msg.lumen
        self.record.mov_mode = msg.mov_mode
        self.record.opr_mode = msg.opr_mode
        self.record.data_log = msg.data_log
        self.record.max_throtle = msg.max_throtle
    def serial_sensor_data_callback(self,msg):
        self.record.imu_yaw = msg.imu_yaw
        self.record.imu_pitch = msg.imu_pitch
        self.record.imu_roll = msg.imu_roll
        self.record.depth = msg.depth
        self.record.pressure_inside = msg.pressure_inside
        self.record.batter_nuc = msg.batter_nuc
        self.record.battery_robot = msg.battery_robot
        self.record.thruster_h_fl = msg.thruster_h_fl
        self.record.thruster_h_fr = msg.thruster_h_fr
        self.record.thruster_h_bl = msg.thruster_h_bl
        self.record.thruster_h_br = msg.thruster_h_br
        self.record.thruster_v_fr = msg.thruster_v_fr
        self.record.thruster_v_fl = msg.thruster_v_fl
        self.record.thruster_v_br = msg.thruster_v_br
        self.record.thruster_v_bl = msg.thruster_v_bl
    def pipeline_cmd_vel_callback(self,msg):
        self.record.pipe_linear.x = msg.linear.x
        self.record.pipe_linear.y = msg.linear.y
        self.record.pipe_linear.z = msg.linear.z
        self.record.pipe_angular.x = msg.angular.x
        self.record.pipe_angular.y = msg.angular.y
        self.record.pipe_angular.z = msg.angular.z
    def master_set_point_callback(self,msg):
        self.record.set_point_yaw = msg.set_point_yaw
        self.record.set_point_pitch = msg.set_point_pitch
        self.record.set_point_roll = msg.set_point_roll
        self.record.set_point_depth = msg.set_point_depth
    def pipeline_detect_callback(self,msg):
        self.record.upper_offset = msg.upper_offset
        self.record.lower_offset = msg.lower_offset

def main(args=None):
    rclpy.init(args=args)
    node=Ping_Node()
    rclpy.spin(node)
    rclpy.shutdown()