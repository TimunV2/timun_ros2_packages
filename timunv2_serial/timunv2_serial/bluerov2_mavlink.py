#import necessary library
import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import pymavlink.dialects.v20.ardupilotmega as mavlink

#import necessary messages
from geometry_msgs.msg import Twist
from timunv2_interfaces.msg import JoyUtilities

class Bluerov2_Mav(Node):
    def __init__(self):
        super().__init__("bluerov2_mavlink_node")
        self.get_logger().info("bluerov2_mavlink_node has been started")

        # Create subscriber
        self.cmd_vel_sub_ = self.create_subscription(Twist, "/master_cmd_vel", self.cmd_vel_callback, 10)
        self.cmd_utl_sub_ = self.create_subscription(JoyUtilities, "/joy_cmd_utl", self.cmd_utl_callback, 10)

        # Setup connection parameters
        self.declare_parameter("ip", "0.0.0.0") 
        self.declare_parameter("port", 14550)
        self.declare_parameter("baudrate", 115200)         

        self.bluerov_ip = self.get_parameter("ip").value
        self.bluerov_port = self.get_parameter("port").value  
        self.bluerov_baudrate = self.get_parameter("baudrate").value

        self.get_logger().info("Controller for bluerov2 was started successfully!")

        # Initialize variable
        # self.type               = mavlink.MAV_TYPE_GCS              # Operator control unit / ground control station.
        # self.autopilot          = mavlink.MAV_AUTOPILOT_INVALID     # No valid autopilot, e.g. a GCS or other MAVLink component.
        # self.base_mode          = mavlink.MAV_MODE_MANUAL_ARMED     # System is not ready to dive, booting, calibrating, etc. No flag is set.
        self.type               = 12                                  # 
        self.autopilot          = 3                                   # 
        self.base_mode          = 81                                  # 
        self.custom_mode        = 19                                  # 
        self.mavlink_version    = 3                                   # MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version.
        self.heartbeat_period   = 0.025

        self.pitch              = 1500                              # The pitch channel (RC1) refers to the upward or downward tilt of the BlueRov2 HEAVY.            
        self.roll               = 1500                              # The roll channel (RC2) is responsible for the horizontal rotation of the BlueRov2 nose around its longitudinal axis.
        self.throttle           = 1500                              # The Throttle channel (RC3) controls the motor power for the diving characteristics of the BlueRov2.
        self.yaw                = 1500                              # The Yaw channel (RC4) is responsible for the rotation of the BlueRov2 nose around the vertical axis.
        self.forward            = 1500                              # The Forward channel (RC5) is responsible for the forward movement of the BlueRov2.
        self.lateral            = 1500                              # The lateral channel (RC6) concerns the lateral movement of the BlueRov2.
        self.camera_pan         = 1500                              # The Camera Pan channel (RC7) controls the horizontal panning movement of the camera.
        self.camera_tilt        = 1500                              # The Camera Tilt channel (RC8) controls the vertical tilt movement of the camera.
        self.lights             = 1100        

        self.arm_hw             = False
        self.arm_sw             = False
        self.depthold          = False

        # Create the connection to the bluerov2
        self.connection = mavutil.mavlink_connection("udpin:" + self.bluerov_ip + ":" + str(self.bluerov_port), baudrate=self.bluerov_baudrate)        
        # self.connection = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)        
        
        self.get_logger().info("Connecting to BlueRov2...")
        self.connection.wait_heartbeat()
        self.get_logger().info("BlueRov2 connection successful!")   

        # convenience
        self.mav        = self.connection.mav
        self.recv_match = self.connection.recv_match
        self.target     = (self.connection.target_system,
                           self.connection.target_component)   

        # Request data stream
        self.get_logger().info("Request data stream...") 
        self.mav.request_data_stream_send(self.connection.target_system, self.connection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL,4, 1)

        # Start update loop for bluerov2 commands. Update frequency are 2Hz
        self.get_logger().info("Start sending heartbeat messages...")
        self.create_timer(self.heartbeat_period, self.send_bluerov_commands) 

    def cmd_vel_callback(self, msg: Twist):
        self.pitch = int(1500+(500*msg.angular.x))
        self.roll = int(1500+(500*msg.angular.y))
        self.yaw = int(1500+(500*msg.angular.z))
        self.forward = int(1500+(500*msg.linear.y))
        self.lateral = int(1500+(500*msg.linear.x))
        self.throttle = int(1500+(500*msg.linear.z))

    def send_bluerov_commands(self):
        # Send regular heartbeats, as a ground control station    
        self.connection.mav.heartbeat_send(self.type, 
                                           self.autopilot, 
                                           self.base_mode, 
                                           self.custom_mode, 
                                           self.mavlink_version)

        # Send RC channel updates
        rc_channel_values = (self.pitch,
                             self.roll,
                             self.throttle,
                             self.yaw,
                             self.forward,
                             self.lateral,
                             self.camera_pan,
                             self.camera_tilt,
                             self.lights,
                             0, 0, 0, 0, 0, 0, 0, 0, 0) # Unused RC channels are ignored        

        self.mav.rc_channels_override_send(*self.target, *rc_channel_values)

    def arm(self):
        self.connection.arducopter_arm()
        self.get_logger().info('Arm requested, waiting...')
        self.connection.motors_armed_wait()
        self.get_logger().info('Thrusters armed!')

    def disarm(self):
        self.connection.arducopter_disarm()
        self.get_logger().info('Disarm requested, waiting...')
        self.connection.motors_disarmed_wait()
        self.get_logger().info('Thrusters disarmed')

    def set_mode(self,mode):
        mode_text ={
            2   :'Depth-Hold',
            19  :'Manual'
        }
        self.get_logger().info(f'{mode_text[mode]} Mode requested, waiting...')
        while not self.connection.wait_heartbeat().custom_mode == mode:
            self.connection.set_mode(mode)
        self.get_logger().info(f'Success Set {mode_text[mode]} Mode')

    def cmd_utl_callback(self, msg: JoyUtilities):
        if msg.arm_hw == True and self.arm_hw == False:
            self.arm()
        elif msg.arm_hw == False and self.arm_hw == True:
            self.disarm()
        self.arm_hw = msg.arm_hw
        if msg.depthhold == True and self.depthold == False:
            self.set_mode(2)
            # pass
        elif msg.depthhold == False and self.depthold == True:
            self.set_mode(19)
            # pass
        self.depthold = msg.depthhold

    

def main(args=None):
    rclpy.init(args=args)
    node=Bluerov2_Mav()
    rclpy.spin(node)
    rclpy.shutdown()