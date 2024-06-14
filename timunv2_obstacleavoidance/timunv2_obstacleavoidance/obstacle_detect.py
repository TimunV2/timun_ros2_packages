import rclpy
import cv2
import numpy as np
import time
import yaml
from rclpy.node import Node
from cv_bridge import CvBridge
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator 
from timunv2_pipefollowing.pid import PID

from sensor_msgs.msg import Image
from timunv2_interfaces.msg import PingData, JoyUtilities, SensorData, SetPoint
from geometry_msgs.msg import Twist

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__("Obstacle_Avoidance_node")
        self.get_logger().info("Obstacle_Avoidance_node has been started")
        self.camera_sub_ = self.create_subscription(Image, "camera_bottom", self.image_callback,10)
        self.ping_sensor_sub_ = self.create_subscription(PingData, "/ping_data", self.ping_callback, 10)
        self.joy_cmd_utl_sub_ = self.create_subscription(JoyUtilities, "/joy_cmd_utl", self.joy_cmd_utl_callback, 10)
        self.serial_sensor_data_sub_ = self.create_subscription(SensorData, "/serial_sensor_data", self.serial_sensor_data_callback, 10)

        self.obs_cmd_vel_pub_ = self.create_publisher(Twist,"/obstacle_cmd_vel",10)
        self.obs_set_point_pub_ = self.create_publisher(SetPoint,"/obstacle_set_point",10)
        self.model = YOLO('/home/ardiar/ws/timunv2_ws/src/timun_ros2_packages/timunv2_obstacleavoidance/timunv2_obstacleavoidance/SelangYOLOV8n.pt')
        self.cv_bridge = CvBridge()
        self.frame = None
        self.frame_area = 0
        self.yaml_filepath = '/home/ardiar/ws/timunv2_ws/src/timun_ros2_packages/timunv2_bringup/config/pidparams.yaml'

        self.timer_ = self.create_timer(0.01, self.timer_callback)
        
        self.pipe_error = 0
        self.pipe_set_point = 0
        self.heading_output = 0
        self.pipe_heading_pid = PID(KP=0.0, KI=0.0, KD=0.0, target=0)
        self.pipe_heading_pid.setLims(min=-1.0, max=1.0)
        self.k_heading = [0, 0, 0]

        self.obs_set_point = SetPoint()
        self.obs_vel = Twist()
        self.status_avoid = 0
        self.coral_area = 0
        self.rock_area = 0
        self.pipe_area = 0
        self.pipe_center_y = 0
        self.last_depth = 0
        self.ranges_scan_last = 0

        self.fps_start_time = time.time()
        self.start_time_move = time.time()
        self.elapsed_time_move = 0

        self.throtle = 0.0
    
    def timer_callback(self):
        self.objectDetect()
        self.publish_messages()

    def joy_cmd_utl_callback(self, msg:JoyUtilities):
        self.opr_mode = msg.opr_mode

    def ping_callback(self, msg:PingData):
        self.ranges_scan = msg.ranges_scan/1000.0
        self.confidence = msg.confidence
        
    def serial_sensor_data_callback(self,msg:SensorData):
        self.imu_yaw = msg.imu_yaw
        self.depth = msg.depth
    
    def publish_messages(self):
        self.obs_set_point_pub_.publish(self.obs_set_point)
        self.obs_cmd_vel_pub_.publish(self.obs_vel)

    def objectDetect(self):
        try:
            if self.opr_mode == 6:
                height , width, channels = self.frame.shape
                self.frame_area = height*width
                center_frame_x = width/2
                center_frame_y = height/2
                # results = self.model(self.frame,max_det=2)
                results = self.model.predict(self.frame, max_det=2)
                annotated_frame = results[0].plot()
                for result in results:
                    annotator = Annotator(self.frame)
                    boxes = result.boxes
                    center_box = []
                    areas = []
                    objects = []
                    for box in boxes:
                        b = box.xyxy[0]
                        b_cy = ((b[3]-b[1])/2)+b[1]
                        b_cx = ((b[2]-b[0])/2)+b[0]
                        c = box.cls                 
                        name = result.names[c.item()]
                        objects.append(name)
                        center_array = [int(b_cx.item()),int(b_cy.item())]
                        center_box.append(center_array)
                        annotator.box_label(b, self.model.names[int(c)])
                        Acy = (b[3]-b[1])
                        Acx = (b[2]-b[0])
                        luasdetect = Acy.item()*Acx.item()
                        areas.append((int(c.item()), luasdetect, int(b_cx.item()))) 
                        print(luasdetect)
                        cv2.circle(annotated_frame, center_array, 2, (0,255,0), 4)
                    for i in areas:
                        if i[0] == 0: #coral
                            self.coral_area = i[1]
                        elif i[0] == 1: #ROCK
                            self.pipe_area = i[1]
                            self.pipe_center_x = i[2]
                        elif i[0] == 2: #ROCK
                            self.rock_area = i[1]
                    if 'coral' not in objects:
                        self.coral_area = 0
                    if 'rock' not in objects:
                        self.rock_area = 0
                    if 'pipe' not in objects:
                        self.pipe_area = 0
                    #============================================== COMMAND STATUS
                    if self.rock_area > 0 or self.coral_area > 0 :
                        if self.rock_area > self.frame_area/16 or self.coral_area > self.frame_area/16:
                            if self.status_avoid != 1: # detect pertama kali
                                self.status_avoid = 1
                                self.obs_vel.linear.x = 0.0
                                self.obs_vel.linear.z = 0.5
                                self.last_depth = self.depth
                                self.get_logger().info('Rock/Coral is Close')
                                self.ranges_scan_last = self.ranges_scan
                                self.get_logger().info('Start Rock/Coral Avoidance Movement...')
                        self.set_point_depth = self.depth - 30 # 30cm offset

                    if self.pipe_area > 0:
                        if self.pipe_area > self.frame_area/16:
                            if self.status_avoid != 3:
                                self.get_logger().info('Pipe is Close')
                                self.get_logger().info('Start Pipe Avoidance Movement...')
                                self.status_avoid = 3
                                self.target_yaw = self.imu_yaw + 180
                                self.obs_vel.linear.x = 0.0
                                self.obs_vel.linear.y = 0.25
                                #======================================= PID PANNING + BRAKING
                                # self.obs_vel.angular.z = 0.25
                                if self.target_yaw > 180 :
                                    self.target_yaw -= 360
                                elif self.target_yaw < -180 :
                                    self.target_yaw += 360
                        if self.status_avoid == 3:
                            self.pipe_offset =  self.pipe_center_x - center_frame_x
                    #============================================== COMMAND GERAKAN
                    if self.status_avoid == 1 : #gerakan menghindar
                        if self.depth >= self.set_point_depth + 10:
                            self.obs_set_point.set_point_depth = self.set_point_depth
                            self.start_time_move = time.time()
                            self.get_logger().info('Goin Up')
                        else:
                            self.obs_set_point.set_point_depth = self.set_point_depth
                            self.obs_vel.linear.x = self.throtle
                            self.obs_vel.linear.z = 0.0
                            if self.ranges_scan < self.ranges_scan_last - 200:
                                self.elapsed_time_move = time.time() - self.start_time_move
                            elif self.ranges_scan > self.ranges_scan_last + 200:
                                self.status_avoid = 2
                        self.ranges_scan_last = self.ranges_scan
                    if self.status_avoid == 2:#gerakan setelah melewati batu
                        self.obs_set_point.set_point_depth = self.depth
                        end_time_move = time.time()
                        self.obs_vel.linear.x = 0.5 
                        if self.elapsed_time_move <= end_time_move - self.start_time_move:
                            self.obs_set_point.set_point_depth = self.last_depth
                            self.obs_vel.linear.x = 0.0
                            self.get_logger().info('Success Avoiding Rock/Coral')
                            self.status_avoid = 0

                    if self.status_avoid == 3:#gerakan menghindar pipa
                        if self.imu_yaw <= self.target_yaw + 5 and self.imu_yaw >= self.target_yaw - 5:
                            self.obs_vel.linear.x = 0.0
                            self.obs_vel.linear.y = 0.0
                            self.obs_vel.angular.z = 0.0
                            self.status_avoid = 4
                    if self.status_avoid == 4: #gerakan memutar kembali ke yaw awal
                        self.obs_set_point.set_point_yaw = self.imu_yaw + 180
                        if self.obs_set_point.set_point_yaw > 180 :
                            self.obs_set_point.set_point_yaw -= 360
                        elif self.obs_set_point.set_point_yaw < -180 :
                            self.obs_set_point.set_point_yaw += 360
                        self.get_logger().info('Success Avoiding Pipe')
                        self.status_avoid = 0

                img = annotator.result()
                elapsed_time = time.time() - self.fps_start_time
                self.frame_count += 1 
                if elapsed_time > 1:  # Update FPS every second
                    fps = self.frame_count / elapsed_time
                    self.fps_start_time = time.time()
                    self.frame_count = 0  # Reset frame counter for next second

                # Display FPS on frame
                fps_text = f"FPS: {fps:.1f}"
                cv2.putText(img, fps_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow("YOLOv8 Tracking", annotated_frame)

        except Exception as e:
            self.get_logger().error(f"Frame is empty: {str(e)}")

    def pid_param_yaml(self):
        try:
            with open(self.yaml_filepath, 'r') as yaml_file:
                data = yaml.safe_load(yaml_file)

            # Extract pid_parameters for serial_node
            obstacle_pid_parameters = data['obstacle_avoidance']['pid_parameters']

            if obstacle_pid_parameters:
                self.k_heading[0] = obstacle_pid_parameters.get('kp_heading', 0)
                self.k_heading[1] = obstacle_pid_parameters.get('ki_heading', 0)
                self.k_heading[2] = obstacle_pid_parameters.get('kd_heading', 0)
                self.k_lateral[0] = obstacle_pid_parameters.get('kp_lateral', 0)
                self.k_lateral[1] = obstacle_pid_parameters.get('ki_lateral', 0)
                self.k_lateral[2] = obstacle_pid_parameters.get('kd_lateral', 0)
                self.throtle = obstacle_pid_parameters.get('throtle', 0)

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

    def image_callback(self, msg):
        try:
            self.frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()