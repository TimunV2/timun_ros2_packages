#import necessary library
import rclpy
from rclpy.node import Node
import time
from timunv2_pipefollowing.pid import PID
import yaml

#import necessary message
from timunv2_interfaces.msg import PipeDetect
from geometry_msgs.msg import Twist

class PipelineNav(Node):
    def __init__(self):
        super().__init__("pipeline_navigation_node")
        self.get_logger().info("pipeline_navigation_node has been started")
        self.pipeline_sub_ = self.create_subscription(PipeDetect, "pipeline_value", self.pipeline_detect_callback, 10)
        self.pipeline_publish_ = self.create_publisher(Twist, "pipeline_cmd_vel", 10)

        self.pid_compute_timer_ = self.create_timer(0.01, self.pid_compute)
        self.pid_update_param_timer_ = self.create_timer(0.1, self.pid_param_yaml)

        self.yaml_filepath = '/home/tkd/timunv2_ws/src/timunv2_bringup/config/pidparams.yaml'

        #pub message variable
        self.pipe_cmd_vel = Twist()

        #heading pid variable
        self.upper_error = 0
        self.upper_set_point = 0
        self.heading_output = 0
        self.pipe_heading_pid = PID(KP=0.0, KI=0.0, KD=0.0, target=0)
        self.pipe_heading_pid.setLims(min=-1.0, max=1.0)
        self.k_heading = [0, 0, 0]

        #lateral pid variable
        self.lower_error = 0
        self.lower_set_point = 0
        self.lateral_output = 0
        self.pipe_lateral_pid = PID(KP=0.001, KI=0.0, KD=0.0, target=0)
        self.pipe_lateral_pid.setLims(min=-1.0, max=1.0)
        self.k_lateral = [0, 0, 0]
        
        #other
        self.throtle = 0.0

    def pipeline_detect_callback(self, msg: PipeDetect):
        self.upper_error = msg.upper_offset
        self.lower_error = msg.lower_offset

    def pid_compute(self):
        self.heading_output = self.pipe_heading_pid.compute(self.upper_error, dt=0.01)
        self.lateral_output = self.pipe_lateral_pid.compute(self.lower_error, dt=0.01)
        # print(f"Heading Output: {self.heading_output}, Lateral Output: {self.lateral_output}")

        self.pipe_cmd_vel.angular.z = float(-1 * self.heading_output)
        self.pipe_cmd_vel.linear.x = float(-1 * self.lateral_output)

        self.pipe_cmd_vel.linear.y = float(self.throtle - abs(self.lower_error/320))
        
        if self.pipe_cmd_vel.linear.y < 0.0:
            self.pipe_cmd_vel.linear.y = 0.0
        else :
            pass
        
        self.pipeline_publish_.publish(self.pipe_cmd_vel)


    def pid_param_yaml(self):
        try:
            with open(self.yaml_filepath, 'r') as yaml_file:
                data = yaml.safe_load(yaml_file)

            # Extract pid_parameters for serial_node
            pipeline_navigation_pid_parameters = data['pipeline_navigation']['pid_parameters']

            if pipeline_navigation_pid_parameters:
                self.k_heading[0] = pipeline_navigation_pid_parameters.get('kp_heading', 0)
                self.k_heading[1] = pipeline_navigation_pid_parameters.get('ki_heading', 0)
                self.k_heading[2] = pipeline_navigation_pid_parameters.get('kd_heading', 0)
                self.k_lateral[0] = pipeline_navigation_pid_parameters.get('kp_lateral', 0)
                self.k_lateral[1] = pipeline_navigation_pid_parameters.get('ki_lateral', 0)
                self.k_lateral[2] = pipeline_navigation_pid_parameters.get('kd_lateral', 0)
                self.throtle = pipeline_navigation_pid_parameters.get('throtle', 0)

                self.pipe_heading_pid.kp = self.k_heading[0]
                self.pipe_heading_pid.ki = self.k_heading[1]
                self.pipe_heading_pid.kd = self.k_heading[2]
                self.pipe_lateral_pid.kp = self.k_lateral[0]
                self.pipe_lateral_pid.ki = self.k_lateral[1]
                self.pipe_lateral_pid.kd = self.k_lateral[2]

            else:
                print("No PID parameters found for serial_node in the YAML file.")
        
        except FileNotFoundError:
            print("YAML file not found.")
        except Exception as e:
            print(f"Error reading YAML file: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = PipelineNav()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()