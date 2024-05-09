import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from timunv2_interfaces.msg import JoyUtilities

mov_button_new = False
mov_button_old = False

class JoySubNode(Node):
    def __init__(self):
        super().__init__("joy_sub_node")
        self.get_logger().info("joy_sub_node has been started")
        self.joy_sub_ = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.joy_cmd_vel_pub_ = self.create_publisher(Twist, "/joy_cmd_vel", 10)
        self.joy_cmd_utl_pub_ = self.create_publisher(JoyUtilities, "/joy_cmd_utl", 10)

        #publish at 100hz
        self.timer_ = self.create_timer(0.01, self.publish_messages)

        #pub message variable
        self.cmd_vel = Twist()
        self.cmd_utl = JoyUtilities()

        #primary variable
        self.lumen_pwr = 0
        self.arm_hardware = False
        self.arm_software = False
        self.movement_mode = 0
        self.operation_mode = 0
        self.data_log = 0
        self.max_throtle_scale = 0.5

        #other variable
        self.sel_button_new = False
        self.sel_button_old = False
        self.str_button_new = False
        self.str_button_old = False
        self.mov_button_new = False
        self.mov_button_old = False
        self.opr_button_new = False
        self.opr_button_old = False
        self.stblz_button_new = False
        self.stblz_button_old = False
        self.dpth_button_new = False
        self.dpth_button_old = False
        self.stabilize = False
        self.depthhold = False
        self.r3_button = False

    def joy_callback(self, msg: Joy):
        #analog sticks to velocity
        self.cmd_vel.linear.x = msg.axes[0]*-1
        self.cmd_vel.linear.y = msg.axes[1]

        alternate_button = msg.buttons[4]

        if alternate_button == 0:
            self.cmd_vel.linear.z = msg.axes[3]
            self.cmd_vel.angular.z = msg.axes[2]*-1
            self.cmd_vel.angular.x = 0.0
            self.cmd_vel.angular.y = 0.0
        elif alternate_button == 1:
            self.cmd_vel.linear.z = 0.0
            self.cmd_vel.angular.z = 0.0
            self.cmd_vel.angular.x = msg.axes[3]
            self.cmd_vel.angular.y = msg.axes[2]*-1

        #"arrow up & down"button as lumen intensity
        if( msg.axes[6] > 0 and self.lumen_pwr < 100 ):
        # if( msg.axes[5] > 0 and self.lumen_pwr < 100 ):
            self.lumen_pwr += 1
        elif( msg.axes[6] < 0 and self.lumen_pwr > 0):
        # elif( msg.axes[5] < 0 and self.lumen_pwr > 0):
            self.lumen_pwr -= 1
        elif( self.lumen_pwr > 100 ):
            self.lumen_pwr = 100
        elif( self.lumen_pwr < 0 ):
            self.lumen_pwr = 0

        #"select" button as hardware arm
        self.sel_button_new = bool(msg.buttons[8])
        if( self.sel_button_new == True and self.sel_button_old == False):
            self.arm_hardware = not(self.arm_hardware)
        self.sel_button_old = self.sel_button_new

        #"start" button as software arm
        self.str_button_new = bool(msg.buttons[9])
        if( self.str_button_new == True and self.str_button_old == False):
            self.arm_software = not(self.arm_software)
        self.str_button_old = self.str_button_new

        #"triangle" button as stabilize mode
        self.stblz_button_new = bool(msg.buttons[3])
        if( self.stblz_button_new == True and self.stblz_button_old == False):
            self.stabilize = not(self.stabilize)
        self.stblz_button_old = self.stblz_button_new

        #"circle" button as depth-hold mode
        self.dpth_button_new = bool(msg.buttons[1])
        if( self.dpth_button_new == True and self.dpth_button_old == False):
            self.depthhold = not(self.depthhold)
        self.dpth_button_old = self.dpth_button_new

        #movement mode
        if(self.stabilize == False and self.depthhold == False):
            self.movement_mode = 0
        elif(self.stabilize == True and self.depthhold == False):
            self.movement_mode = 1
        elif(self.stabilize == False and self.depthhold == True):
            self.movement_mode = 2
        elif(self.stabilize == True and self.depthhold == True):
            self.movement_mode = 3

        #"square" button as operation mode
        self.opr_button_new = bool(msg.buttons[2])
        if( self.opr_button_new == True and self.opr_button_old == False):
            self.operation_mode += 1
            if(self.operation_mode > 4):
                self.operation_mode = 0
        self.opr_button_old = self.opr_button_new

        #r3 button as IMU reset
        self.r3_button = bool(msg.buttons[11])

    def publish_messages(self):
        self.cmd_utl.lumen = self.lumen_pwr
        self.cmd_utl.arm_hw = self.arm_hardware
        self.cmd_utl.arm_sw = self.arm_software
        self.cmd_utl.mov_mode = self.movement_mode
        self.cmd_utl.opr_mode = self.operation_mode
        self.cmd_utl.stabilize = self.stabilize
        self.cmd_utl.depthhold = self.depthhold
        self.cmd_utl.max_throtle = self.max_throtle_scale
        self.cmd_utl.imu_reset = self.r3_button

        self.joy_cmd_vel_pub_.publish(self.cmd_vel)
        self.joy_cmd_utl_pub_.publish(self.cmd_utl)


def main(args=None):
    rclpy.init(args=args)
    node = JoySubNode()
    rclpy.spin(node)
    rclpy.shutdown()