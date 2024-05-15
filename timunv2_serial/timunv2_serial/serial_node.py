#import necessary library
import rclpy
from rclpy.node import Node
import serial
import struct
import os
import yaml

#import necessary messages
from geometry_msgs.msg import Twist
from timunv2_interfaces.msg import SetPoint
from timunv2_interfaces.msg import JoyUtilities
from timunv2_interfaces.msg import SensorData

class Serial_Node(Node):
    def __init__(self):
        super().__init__("serial_node")
        self.get_logger().info("serial_node has been started")
        #subscriber
        self.master_cmd_vel_sub_ = self.create_subscription(Twist, "/master_cmd_vel", self.master_cmd_vel_callback, 10)
        self.master_set_point_sub_ = self.create_subscription(SetPoint, "/master_set_point", self.master_set_point_callback, 10)
        # self.pid_const_sub_ = self.create_subscription(PIDConstant, "/pid_constant", self.pid_const_callback, 10)
        #publisher
        self.joy_cmd_utl_sub_ = self.create_subscription(JoyUtilities, "/joy_cmd_utl", self.joy_cmd_utl_callback, 10)
        self.serial_sensor_pub_ = self.create_publisher(SensorData, "/serial_sensor_data", 10)

        self.timer_ = self.create_timer(0.01, self.timer_callback)

        self.yaml_filepath = '/home/tkd/timunv2_ws/src/timunv2_bringup/config/pidparams.yaml'

        #Serial 
        self.port = '/dev/ttyACM1'
        self.baudrate = 115200
        self.ser = None
        self.communication_status_now = False
        self.communication_status_last = False
        self.write_status_now = False
        self.write_status_last = True
        self.read_status_now = False
        self.read_status_last = True

        #velocity converted          x, y, z
        self.vel_linear_converted = [0, 0, 0]
        self.vel_angular_converted = [0, 0, 0]
        self.max_throtle_scale = 0.0
        self.max_throtle_scale_converted = 0

        #setpint converted          yaw pitch roll
        self.set_point_converted = [0, 0, 0, 0]

        #pid constant kp, ki, kd
        self.k_yaw = [0, 0, 0]
        self.k_pitch = [0, 0, 0]
        self.k_roll = [0, 0, 0]
        self.k_depth = [0, 0, 0]

        #utilities command variable
        self.lumen_pwr = 0
        self.arm_hardware = False
        self.arm_software = False
        self.movement_mode = 0
        self.operation_mode = 0
        self.imu_reset = False

        #sensor data variable
        self.sensor = SensorData()

    def master_cmd_vel_callback(self, msg: Twist):
        self.vel_linear_converted[0] = int(500*self.max_throtle_scale*msg.linear.x)
        self.vel_linear_converted[1] = int(500*self.max_throtle_scale*msg.linear.y)
        self.vel_linear_converted[2] = int(500*self.max_throtle_scale*msg.linear.z)
        self.vel_angular_converted[0] = int(500*self.max_throtle_scale*msg.angular.x)
        self.vel_angular_converted[1] = int(500*self.max_throtle_scale*msg.angular.y)
        self.vel_angular_converted[2] = int(500*self.max_throtle_scale*msg.angular.z)

    def master_set_point_callback(self, msg: SetPoint):
        self.set_point_converted[0] = int(msg.set_point_yaw)
        self.set_point_converted[1] = int(msg.set_point_pitch)
        self.set_point_converted[2] = int(msg.set_point_roll)
        self.set_point_converted[3] = int(msg.set_point_depth)

    def joy_cmd_utl_callback(self, msg: JoyUtilities):
        self.max_throtle_scale = msg.max_throtle
        self.max_throtle_scale_converted = int(500*self.max_throtle_scale)
        self.lumen_pwr = msg.lumen
        self.arm_hardware = msg.arm_hw
        self.arm_software = msg.arm_sw
        self.movement_mode = msg.mov_mode
        self.operation_mode = msg.opr_mode
        self.imu_reset = msg.imu_reset

    def pid_const_yaml(self):
        try:
            with open(self.yaml_filepath, 'r') as yaml_file:
                data = yaml.safe_load(yaml_file)

            # Extract pid_parameters for serial_node
            serial_node_pid_parameters = data['serial_node']['pid_parameters']

            if serial_node_pid_parameters:
                self.k_yaw[0] = serial_node_pid_parameters.get('kp_yaw', 0)
                self.k_yaw[1] = serial_node_pid_parameters.get('ki_yaw', 0)
                self.k_yaw[2] = serial_node_pid_parameters.get('kd_yaw', 0)
                self.k_pitch[0] = serial_node_pid_parameters.get('kp_pitch', 0)
                self.k_pitch[1] = serial_node_pid_parameters.get('ki_pitch', 0)
                self.k_pitch[2] = serial_node_pid_parameters.get('kd_pitch', 0)
                self.k_roll[0] = serial_node_pid_parameters.get('kp_roll', 0)
                self.k_roll[1] = serial_node_pid_parameters.get('ki_roll', 0)
                self.k_roll[2] = serial_node_pid_parameters.get('kd_roll', 0)
                self.k_depth[0] = serial_node_pid_parameters.get('kp_depth', 0)
                self.k_depth[1] = serial_node_pid_parameters.get('ki_depth', 0)
                self.k_depth[2] = serial_node_pid_parameters.get('kd_depth', 0)
            else:
                print("No PID parameters found for serial_node in the YAML file.")
        
        except FileNotFoundError:
            print("YAML file not found.")
        except Exception as e:
            print(f"Error reading YAML file: {str(e)}")

    def serial_write(self):
        def float_to_int16(value):
            # Scale the float value and convert to integer
            scaled_value = int(value * 100)  # Scale by 100 for precision, adjust as needed
            # Ensure the value fits into a signed short integer range (-32,768 to 32,767)
            if scaled_value < -32768:
                scaled_value = -32768
            elif scaled_value > 32767:
                scaled_value = 32767
            return scaled_value
        # 0
        message_linear_x = self.vel_linear_converted[0].to_bytes(2, byteorder='big', signed=True)
        message_linear_y = self.vel_linear_converted[1].to_bytes(2, byteorder='big', signed=True)
        message_linear_z = self.vel_linear_converted[2].to_bytes(2, byteorder='big', signed=True)
        message_angular_x = self.vel_angular_converted[0].to_bytes(2, byteorder='big', signed=True)
        message_angular_y = self.vel_angular_converted[1].to_bytes(2, byteorder='big', signed=True)
        message_angular_z = self.vel_angular_converted[2].to_bytes(2, byteorder='big', signed=True)
        # 12
        message_throtle_scale = self.max_throtle_scale_converted.to_bytes(2, byteorder='big', signed=True)
        # 14
        message_set_point_yaw = self.set_point_converted[0].to_bytes(2, byteorder='big', signed=True)
        message_set_point_pitch = self.set_point_converted[1].to_bytes(2, byteorder='big', signed=True)
        message_set_point_roll = self.set_point_converted[2].to_bytes(2, byteorder='big', signed=True)
        message_set_point_depth = self.set_point_converted[3].to_bytes(2, byteorder='big', signed=True)
        # 22
        # message_kp_yaw = self.k_yaw[0].to_bytes(2, byteorder='big', signed=True)
        # message_ki_yaw = self.k_yaw[1].to_bytes(2, byteorder='big', signed=True)
        # message_kd_yaw = self.k_yaw[2].to_bytes(2, byteorder='big', signed=True)
        # message_kp_pitch = self.k_pitch[0].to_bytes(2, byteorder='big', signed=True)
        # message_ki_pitch = self.k_pitch[1].to_bytes(2, byteorder='big', signed=True)
        # message_kd_pitch = self.k_pitch[2].to_bytes(2, byteorder='big', signed=True)
        # message_kp_roll = self.k_roll[0].to_bytes(2, byteorder='big', signed=True)
        # message_ki_roll = self.k_roll[1].to_bytes(2, byteorder='big', signed=True)
        # message_kd_roll = self.k_roll[2].to_bytes(2, byteorder='big', signed=True)
        # message_kp_depth = self.k_depth[0].to_bytes(2, byteorder='big', signed=True)
        # message_ki_depth = self.k_depth[1].to_bytes(2, byteorder='big', signed=True)
        # message_kd_depth = self.k_depth[2].to_bytes(2, byteorder='big', signed=True)
        # Convert floating-point numbers to two-byte integers and then to bytes
        message_kp_yaw = struct.pack('>h', float_to_int16(self.k_yaw[0]))
        message_ki_yaw = struct.pack('>h', float_to_int16(self.k_yaw[1]))
        message_kd_yaw = struct.pack('>h', float_to_int16(self.k_yaw[2]))
        message_kp_pitch = struct.pack('>h', float_to_int16(self.k_pitch[0]))
        message_ki_pitch = struct.pack('>h', float_to_int16(self.k_pitch[1]))
        message_kd_pitch = struct.pack('>h', float_to_int16(self.k_pitch[2]))
        message_kp_roll = struct.pack('>h', float_to_int16(self.k_roll[0]))
        message_ki_roll = struct.pack('>h', float_to_int16(self.k_roll[1]))
        message_kd_roll = struct.pack('>h', float_to_int16(self.k_roll[2]))
        message_kp_depth = struct.pack('>h', float_to_int16(self.k_depth[0]))
        message_ki_depth = struct.pack('>h', float_to_int16(self.k_depth[1]))
        message_kd_depth = struct.pack('>h', float_to_int16(self.k_depth[2]))
        # 46
        message_lumen_pwr = self.lumen_pwr.to_bytes(2, byteorder='big', signed=True)
        # 48
        message_movement_mode = self.movement_mode.to_bytes(1, byteorder='big', signed=True)
        message_operation_mode = self.operation_mode.to_bytes(1, byteorder='big', signed=True)
        message_arm_hardware = self.arm_hardware.to_bytes(1, byteorder='big', signed=True)
        message_arm_software = self.arm_software.to_bytes(1, byteorder='big', signed=True)
        message_imu_reset = self.imu_reset.to_bytes(1, byteorder='big', signed=True)
        
        try:
            self.ser = serial.Serial(self.port, self.baudrate)
            self.communication_status_now = True
            if self.communication_status_now == True and self.communication_status_last == False:
                self.get_logger().info("Serial port opening success")
        
        except Exception as e:
            self.communication_status_now = False
            if self.communication_status_now == False and self.communication_status_last == True:
                self.get_logger().error(f"Error opening serial port: {str(e)}")
            
        self.communication_status_last = self.communication_status_now

        try:
            self.ser.write(message_linear_x + message_linear_y + message_linear_z
                                + message_angular_x + message_angular_y + message_angular_z
                                + message_throtle_scale
                                + message_set_point_yaw + message_set_point_pitch + message_set_point_roll + message_set_point_depth
                                + message_kp_yaw + message_ki_yaw + message_kd_yaw
                                + message_kp_pitch + message_ki_pitch + message_kd_pitch
                                + message_kp_roll + message_ki_roll + message_kd_roll
                                + message_kp_depth + message_ki_depth + message_kd_depth
                                + message_lumen_pwr
                                + message_movement_mode + message_operation_mode
                                + message_arm_hardware + message_arm_software
                                + message_imu_reset
                                )
            self.write_status_now = True

        except Exception as e:
            self.write_status_now = False
            if self.write_status_now == False and self.write_status_last == True:
                self.get_logger().error(f"Error writing serial port: {str(e)}")

        self.write_status_last = self.write_status_now
        
        # print("terkirim")
        # 0 - 11 Velocity
        # 12 - 13 Throtle Scale
        # 14 - 21 Setpoint
        # 22 - 23 Lumen Power
        # 24 - 25 Mode
        # 26 - 27 Arming Toggle 

    def serial_read(self):
        try:
            received_data = self.ser.read(14)
            if received_data:
                received_yaw, received_pitch, received_roll, received_depth, received_pressure, received_batt1, received_batt2  = struct.unpack('hhhhhhh', received_data)
                self.sensor.imu_yaw = received_yaw/10.0
                self.sensor.imu_pitch = received_pitch/10.0
                self.sensor.imu_roll= received_roll/10.0
                self.sensor.depth = received_depth/100.0
                self.sensor.pressure_inside = received_pressure/100.0
                self.sensor.batter_nuc = received_batt1/100.0
                self.sensor.battery_robot = received_batt2/100.0

            self.read_status_now = True

        except Exception as e:
            self.read_status_now = False
            if self.read_status_now == False and self.read_status_last == True:
                self.get_logger().error(f"Error reading serial port: {str(e)}")

        self.read_status_last = self.read_status_now
        

    def timer_callback(self):
        # self.get_logger().info("enter timer callback")
        self.pid_const_yaml()
        self.serial_write()
        self.serial_read()
        self.serial_sensor_pub_.publish(self.sensor)

def main(args=None):
    rclpy.init(args=args)
    node=Serial_Node()
    rclpy.spin(node)
    rclpy.shutdown()