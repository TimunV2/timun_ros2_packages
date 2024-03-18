#import necessary library
import rclpy
from rclpy.node import Node

#import necessary messages
from geometry_msgs.msg import Twist
from timunv2_interfaces.msg import JoyUtilities
from timunv2_interfaces.msg import SetPoint
from timunv2_interfaces.msg import SensorData

class MasterController(Node):
    def __init__(self):
        super().__init__("master_controller")
        self.get_logger().info("master_controller node has been started")
        #init subscriber
        self.joy_cmd_vel_sub_ = self.create_subscription(Twist, "/joy_cmd_vel", self.joy_cmd_vel_callback, 10)
        self.joy_cmd_utl_sub_ = self.create_subscription(JoyUtilities, "/joy_cmd_utl", self.joy_cmd_utl_callback, 10)
        self.serial_sensor_data_sub_ = self.create_subscription(SensorData, "/serial_sensor_data", self.serial_sensor_data_callback, 10)
        self.pipeline_cmd_vel_sub_ = self.create_subscription(Twist, "/pipeline_cmd_vel", self.pipeline_cmd_vel_callback, 10)
        
        #init publisher
        self.master_cmd_vel_pub_ = self.create_publisher(Twist, "/master_cmd_vel", 10)
        self.master_setpoint_pub_ = self.create_publisher(SetPoint, "master_set_point", 10)

        self.timer_ = self.create_timer(0.01, self.timer_callback)  # 0.01 seconds (100 Hz)

        #velocity command variable
        self.joy_cmd_vel = Twist()
        self.pipe_cmd_vel = Twist()
        self.master_cmd_vel = Twist()
        self.temp_cmd_vel = Twist()

        #utilities command variable
        self.lumen_pwr = 0
        self.arm_hardware = False
        self.arm_software = False
        self.movement_mode = 0
        self.operation_mode = 0
        self.data_log = 0
        self.stabilize = False
        self.depthold = False

        #sensor variable
        self.imu_yaw = 0.0
        self.imu_pitch = 0.0
        self.imu_roll = 0.0
        self.depth_value = 0.0
        self.sonar_ranges = 0.0
        self.sonar_confidence = 0.0

        #movement set-point
        self.yaw_setpoint = 0.0
        self.pitch_setpoint = 0.0
        self.roll_setpoint = 0.0
        self.depth_setpoint = 0.0
        self.master_setpoint = SetPoint()

        #movement mode variable
        self.stabilize_state_new = False
        self.stabilize_state_old = False
        self.depthhold_state_new = False
        self.depthhold_state_old = False

        #set point drift sensitivity 
        self.yaw_drift_scale = 1.0
        self.pitch_drift_scale = 1.0
        self.roll_drift_scale = 1.0
        self.depth_drift_scale = 1.0

        #angular vel sensitivity   X , Y ,  Z
        self.cmd_angular_scale = [1.0, 1.0, 1.0]

        #operation mode variable
        self.operation_mode_auto = False

    def joy_cmd_vel_callback(self, msg: Twist):
        self.joy_cmd_vel.linear.x = msg.linear.x
        self.joy_cmd_vel.linear.y = msg.linear.y
        self.joy_cmd_vel.linear.z = msg.linear.z
        self.joy_cmd_vel.angular.x = msg.angular.x
        self.joy_cmd_vel.angular.y = msg.angular.y
        self.joy_cmd_vel.angular.z = msg.angular.z
        
    def joy_cmd_utl_callback(self, msg: JoyUtilities):
        self.arm_software = msg.arm_sw
        self.movement_mode = msg.mov_mode
        self.operation_mode = msg.opr_mode
        self.data_log = msg.data_log
        self.stabilize = msg.stabilize
        self.depthold = msg.depthhold

    def serial_sensor_data_callback(self, msg: SensorData):
        self.imu_yaw = msg.imu_yaw
        self.imu_pitch = msg.imu_pitch
        self.imu_roll = msg.imu_roll
        self.depth_value = msg.depth
        self.sonar_ranges = msg.ranges_scan
        self.sonar_confidence = msg.confidence

    def pipeline_cmd_vel_callback(self, msg: Twist):
        self.pipe_cmd_vel.linear.x = msg.linear.x
        self.pipe_cmd_vel.linear.y = msg.linear.y
        self.pipe_cmd_vel.linear.z = msg.linear.z
        self.pipe_cmd_vel.angular.z = msg.angular.z

    def movement_mode_(self):
        self.stabilize_state_new = self.stabilize
        self.depthhold_state_new = self.depthold

        #capture actual
        #============================================================================
        if(self.stabilize_state_new == True and self.stabilize_state_old == False):
            #capture current orientation value and make it as set-point
            self.yaw_setpoint = self.imu_yaw
            self.pitch_setpoint = self.imu_pitch
            self.roll_setpoint = self.imu_roll
        self.stabilize_state_old = self.stabilize_state_new
        
        if(self.depthhold_state_new == True and self.depthhold_state_old == False):
            #capture current depth value and make it as set-point
            self.depth_setpoint = self.depth_value
        self.depthhold_state_old = self.depthhold_state_new
        #============================================================================

        if self.movement_mode == 0: #fully manual
            #all velocity command direct from joy input
            self.master_cmd_vel.linear.x = self.joy_cmd_vel.linear.x
            self.master_cmd_vel.linear.y = self.joy_cmd_vel.linear.y
            self.master_cmd_vel.linear.z = self.joy_cmd_vel.linear.z
            self.master_cmd_vel.angular.x = self.joy_cmd_vel.angular.x*self.cmd_angular_scale[0]
            self.master_cmd_vel.angular.y = self.joy_cmd_vel.angular.y*self.cmd_angular_scale[1]
            self.master_cmd_vel.angular.z = self.joy_cmd_vel.angular.z*self.cmd_angular_scale[2]
        
        elif self.movement_mode == 1: #stabilize
            #linear velocity direct from joy input
            #angular velocity replace by driting the orientation pid set point 
            self.master_cmd_vel.linear.x = self.joy_cmd_vel.linear.x
            self.master_cmd_vel.linear.y = self.joy_cmd_vel.linear.y
            self.master_cmd_vel.linear.z = self.joy_cmd_vel.linear.z
            #drift_setpoint_orientation
            self.yaw_setpoint += self.yaw_drift_scale*self.joy_cmd_vel.angular.z
            self.pitch_setpoint += self.pitch_drift_scale*self.joy_cmd_vel.angular.x
            self.roll_setpoint += self.roll_drift_scale*self.joy_cmd_vel.angular.y

        elif self.movement_mode == 2: #depthhold
            #linear x,y velocity direct from joy input
            #angular velocity direct from joy input
            self.master_cmd_vel.linear.x = self.joy_cmd_vel.linear.x
            self.master_cmd_vel.linear.y = self.joy_cmd_vel.linear.y
            #linear z velocity replace by driting the depth pid set point
            self.master_cmd_vel.angular.x = self.joy_cmd_vel.angular.x*self.cmd_angular_scale[0]
            self.master_cmd_vel.angular.y = self.joy_cmd_vel.angular.y*self.cmd_angular_scale[1]
            self.master_cmd_vel.angular.z = self.joy_cmd_vel.angular.z*self.cmd_angular_scale[2]
            #drift_setpoint_depth
            self.depth_setpoint += self.depth_drift_scale*self.joy_cmd_vel.linear.z
        
        elif self.movement_mode == 3: #fully assisted
            #from operation mode
            self.depth_setpoint += self.depth_drift_scale*self.temp_cmd_vel.linear.z
            self.yaw_setpoint += self.yaw_drift_scale*self.temp_cmd_vel.angular.z
            self.pitch_setpoint += self.pitch_drift_scale*self.temp_cmd_vel.angular.x
            self.roll_setpoint += self.roll_drift_scale*self.temp_cmd_vel.angular.y

        if self.yaw_setpoint > 180 :
            self.yaw_setpoint -= 360
        elif self.yaw_setpoint < -180 :
            self.yaw_setpoint += 360
        if self.pitch_setpoint > 180 :
            self.pitch_setpoint -= 360
        elif self.pitch_setpoint < -180 :
            self.pitch_setpoint += 360
        if self.roll_setpoint > 180 :
            self.roll_setpoint -= 360
        elif self.roll_setpoint < -180 :
            self.roll_setpoint += 360
        if self.depth_setpoint > 0.0 :
            self.depth_setpoint = 0.0

    def operation_mode_(self):
        # Set point drift :
        # Use temp_cmd_vel when pairing to timunv2 fc
        # Use master_cmd_vel when pairing to bluerov2
        if self.movement_mode == 3 and self.operation_mode == 0: #manual operation (All input from joy)
            #throtle and lateral from joy
            self.master_cmd_vel.linear.x = self.joy_cmd_vel.linear.x
            self.master_cmd_vel.linear.y = self.joy_cmd_vel.linear.y

            # Temp / Master Changes
            #setpoint drift from joy
            self.master_cmd_vel.linear.z = self.joy_cmd_vel.linear.z
            self.master_cmd_vel.angular.x =self.joy_cmd_vel.angular.x
            self.master_cmd_vel.angular.y =self.joy_cmd_vel.angular.y
            self.master_cmd_vel.angular.z =self.joy_cmd_vel.angular.z

        elif self.movement_mode == 3 and self.operation_mode == 1: #auto 1 (Pipefoll 1 Heading >> cv, Lateral >> joy, Throtle >> joy)
            #throtle and lateral from joy
            self.master_cmd_vel.linear.x = self.joy_cmd_vel.linear.x
            self.master_cmd_vel.linear.y = self.joy_cmd_vel.linear.y

            # Temp / Master Changes
            #depth setpoint drift from joy
            self.master_cmd_vel.linear.z = self.joy_cmd_vel.linear.z
            #yaw setpoint drift from cv
            self.master_cmd_vel.angular.z =self.pipe_cmd_vel.angular.z

        elif self.movement_mode == 3 and self.operation_mode == 2: #auto 2 (Pipefoll 2 Heading >> joy, Lateral >> cv, Throtle >> joy)
            #lateral from cv
            self.master_cmd_vel.linear.x = self.pipe_cmd_vel.linear.x
            #throtle from joy
            self.master_cmd_vel.linear.y = self.joy_cmd_vel.linear.y

            # Temp / Master Changes
            #depth setpoint drift from joy
            self.master_cmd_vel.linear.z = self.joy_cmd_vel.linear.z
            #yaw setpoint drift from joy
            self.master_cmd_vel.angular.z =self.joy_cmd_vel.angular.z

        elif self.movement_mode == 3 and self.operation_mode == 3: #auto 3 (Pipefoll 3 Heading >> cv, Lateral >> cv, Throtle >> joy)
            #lateral from cv
            self.master_cmd_vel.linear.x = self.pipe_cmd_vel.linear.x
            #throtle from joy
            self.master_cmd_vel.linear.y = self.joy_cmd_vel.linear.y

            # Temp / Master Changes
            #depth setpoint drift from joy
            self.master_cmd_vel.linear.z = self.joy_cmd_vel.linear.z
            #yaw setpoint drift from cv
            self.master_cmd_vel.angular.z =self.pipe_cmd_vel.angular.z

        elif self.movement_mode == 3 and self.operation_mode == 4: #auto 4 (Pipefoll 4 Heading >> cv, Lateral >> cv, Throtle >> cv)
            #lateral from cv
            self.master_cmd_vel.linear.x = self.pipe_cmd_vel.linear.x
            #throtle from joy
            self.master_cmd_vel.linear.y = self.pipe_cmd_vel.linear.y

            # Temp / Master Changes
            #depth setpoint drift from joy
            self.master_cmd_vel.linear.z = self.joy_cmd_vel.linear.z
            #yaw setpoint drift from joy
            self.master_cmd_vel.angular.z =self.pipe_cmd_vel.angular.z

        if self.arm_software == False :
            self.master_cmd_vel.linear.x = 0.0
            self.master_cmd_vel.linear.y = 0.0
            self.master_cmd_vel.linear.z = 0.0
            self.master_cmd_vel.angular.x = 0.0
            self.master_cmd_vel.angular.y = 0.0
            self.master_cmd_vel.angular.z = 0.0

        elif self.arm_software == True :
            pass


    def mixed_vel_cmd(self):
        pass

    def timer_callback(self):
        #call other loop at 100hz
        self.movement_mode_()
        self.operation_mode_()
        self.mixed_vel_cmd()
        self.publish_messages()

    def publish_messages(self):
        self.master_setpoint.set_point_yaw = self.yaw_setpoint
        self.master_setpoint.set_point_pitch = self.pitch_setpoint
        self.master_setpoint.set_point_roll = self.roll_setpoint
        self.master_setpoint.set_point_depth = self.depth_setpoint

        self.master_cmd_vel_pub_.publish(self.master_cmd_vel)
        self.master_setpoint_pub_.publish(self.master_setpoint)

def main(args=None):
    rclpy.init(args=args)
    node=MasterController()
    rclpy.spin(node)
    rclpy.shutdown()