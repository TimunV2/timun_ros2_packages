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
        self.camera_sub_ = self.create_subscription(Image, "/camera_front", self.image_callback,10)
        # self.ping_sensor_sub_ = self.create_subscription(PingData, "/ping_data", self.ping_callback, 10)
        self.joy_cmd_utl_sub_ = self.create_subscription(JoyUtilities, "/joy_cmd_utl", self.joy_cmd_utl_callback, 10)
        self.serial_sensor_data_sub_ = self.create_subscription(SensorData, "/serial_sensor_data", self.serial_sensor_data_callback, 10)

        self.obs_cmd_vel_pub_ = self.create_publisher(Twist,"/obstacle_cmd_vel",10)
        self.obs_set_point_pub_ = self.create_publisher(SetPoint,"/obstacle_set_point",10)
        self.model = YOLO('/home/tkd/timunv2_ws/src/timunv2_obstacleavoidance/timunv2_obstacleavoidance/segmentasi-kedua.pt')
        # self.model = YOLO('/home/tkd/timunv2_ws/src/timunv2_obstacleavoidance/timunv2_obstacleavoidance/model-od-14juni.pt')
        self.cv_bridge = CvBridge()
        self.frame = None
        self.frame_area = 0
        # self.yaml_filepath = '/home/tkd/timunv2_ws/src/timunv2_bringup/config/pidparams.yaml'
        self.yaml_filepath = '/home/tkd/timunv2_ws/src/timunv2_bringup/config/oaparams.yaml'

        self.throtle = [0.0, 0.0, 0.0, 0.0]
        self.duration = [0, 0, 0]

        self.timer_ = self.create_timer(0.01, self.timer_callback)
        
        self.frame_count = 0
        self.fps = 0
        

        self.avoid_mode = 0 # 0 for vertical mode, 1 for horizontal mode
        self.obs_set_point = SetPoint()
        self.obs_vel = Twist()
        self.status_avoid = 0
        self.coral_area = 0
        self.rock_area = 0
        self.pipe_area = 0
        self.pipe_center_y = 0
        self.last_depth = 0
        self.ranges_scan_last = 0.0
        self.ranges_scan = 0.6
        self.opr_mode = 0
        self.delay = 6

        self.fps_start_time = time.time()
        self.start_time_move = time.time()
        self.elapsed_time_move = 0

        self.avoid_step = 0
        self.target_move_time = 0
    
    def timer_callback(self):
        self.objectDetect()
        self.publish_messages()
        self.param_yaml()

    def joy_cmd_utl_callback(self, msg:JoyUtilities):
        self.opr_mode = msg.opr_mode

    def ping_callback(self, msg:PingData):
        # self.ranges_scan = msg.ranges_scan/1000.0
        self.ranges_scan = 0.6
        # self.confidence = msg.confidence
        
    def serial_sensor_data_callback(self,msg:SensorData):
        self.imu_yaw = msg.imu_yaw
        self.depth = msg.depth
    
    def publish_messages(self):
        self.obs_set_point_pub_.publish(self.obs_set_point)
        self.obs_cmd_vel_pub_.publish(self.obs_vel)
        # self.obs_set_point_pub_.publish(self.obs_set_point)
        # self.obs_cmd_vel_pub_.publish(self.obs_vel)

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
                            # self.coral_area = i[1]
                            self.coral_area = 0
                        elif i[0] == 2: #ROCK
                            self.rock_area = i[1]
                        elif i[0] == 1: #ROCK
                            self.pipe_area = i[1]
                            self.pipe_center_x = i[2]
                    # if 'selang' not in objects:
                    #     self.coral_area = 0
                    if 'coral' not in objects:
                        self.coral_area = 0
                    if 'rock' not in objects:
                        self.rock_area = 0
                    if 'pipe' not in objects:
                        self.pipe_area = 0
                    #============================================== COMMAND STATUS
                self.get_logger().info(f'area rock{self.rock_area}')
                if self.rock_area > 0 or self.coral_area > 0 :
                    if self.rock_area > self.frame_area/10 or self.coral_area > self.frame_area/10:
                        if self.avoid_mode == 0 :
                            if self.status_avoid != 1: # detect pertama kali
                                self.status_avoid = 1
                                self.obs_vel.linear.y = 0.0
                                # self.obs_vel.linear.z = 0.5
                                self.last_depth = self.depth
                                self.get_logger().info('Rock/Coral is Close')
                                self.ranges_scan_last = self.ranges_scan
                                self.obs_set_point.set_point_depth = self.set_point_depth
                            self.set_point_depth = self.depth - 50 # 30cm offset
                            self.get_logger().info('Start Rock/Coral Vertical Avoidance Movement...')

                        elif self.avoid_mode == 1 :
                            if self.status_avoid != 1: # detect pertama kali
                                self.get_logger().info('pipe is Close')
                                self.status_avoid = 1
                                self.target_move_time = time.time() + self.duration[0] # bergeser selama durasi 0 
                            self.get_logger().info('Start Rock/Coral Horizontal Avoidance Movement...')

                # if self.pipe_area > 0:
                #     if self.pipe_area > self.frame_area/16:
                #         if self.status_avoid != 3:
                #             self.get_logger().info('Pipe is Close')
                #             self.get_logger().info('Start Pipe Avoidance Movement...')
                #             self.status_avoid = 3
                #             # self.target_yaw = self.imu_yaw + 180
                #             self.obs_vel.linear.y = 0.0
                #             self.obs_vel.linear.x = 0.25
                #             #======================================= PID PANNING + BRAKING
                #             # self.obs_vel.angular.z = 0.25
                #             # if self.target_yaw > 180 :
                #             #     self.target_yaw -= 360
                #             # elif self.target_yaw < -180 :
                #             #     self.target_yaw += 360
                #     if self.status_avoid == 3:
                #         self.pipe_offset =  self.pipe_center_x - center_frame_x


                #============================================== COMMAND GERAKAN
                if self.avoid_mode == 0 : # vertical avoid
                    if self.status_avoid == 1 : #gerakan menghindar
                        if self.depth >= self.set_point_depth+20:
                            self.obs_set_point.set_point_depth = self.set_point_depth
                            self.obs_vel.linear.z = 0.5
                            self.obs_vel.linear.y = 0.0
                            self.start_time_move = time.time()
                            self.get_logger().info(f'set_point_depth {self.set_point_depth}')
                            self.get_logger().info('Goin Up')
                        else:
                            self.get_logger().info('Start Moving Forward')
                            self.obs_set_point.set_point_depth = self.set_point_depth
                            self.obs_vel.linear.y = 0.4
                            self.obs_vel.linear.z = 0.0
                            # if self.ranges_scan < self.ranges_scan_last - 100:
                            #     self.elapsed_time_move = time.time() - self.start_time_move
                            # elif self.ranges_scan > self.ranges_scan_last + 100:
                            #     self.get_logger().info('getting elapsed time')
                            #     self.status_avoid = 2
                            end_time_move = time.time()
                            if self.delay <= end_time_move - self.start_time_move:
                                self.start_time_move = time.time()
                                self.status_avoid = 2
                        self.ranges_scan_last = self.ranges_scan
                    elif self.status_avoid == 2:#gerakan setelah melewati batu
                        # self.obs_set_point.set_point_depth = self.depth
                        end_time_move = time.time()
                        # self.obs_vel.linear.x = 0.5 
                        if self.delay <= end_time_move - self.start_time_move:
                            self.obs_set_point.set_point_depth = self.last_depth
                            self.obs_vel.linear.y = 0.0
                            self.get_logger().info('Success Avoiding Rock/Coral')
                            self.status_avoid = 0
                            self.avoid_mode = 1
                    elif self.status_avoid == 0:
                        self.obs_vel.linear.y = 0.6
                        self.obs_set_point.set_point_depth = self.depth
                    else:
                        self.obs_vel.linear.y = 0.0
                        self.obs_vel.linear.z = 0.0
                
                elif self.avoid_mode == 1: # horizontal avoid
                    if self.status_avoid == 0 : # maju
                        self.get_logger().info('Moving Forward')
                        self.obs_vel.linear.y = self.throtle[0]
                        self.obs_vel.linear.x = 0.0
                        # self.obs_set_point.set_point_depth = self.depth

                    elif self.status_avoid == 1 : # gerakan menghindar
                        #kekanan
                        if time.time() < self.target_move_time and self.avoid_step == 0:
                            self.get_logger().info('Moving Right')
                            self.obs_vel.linear.y = 0.0
                            self.obs_vel.linear.x = self.throtle[1]
                        elif time.time() >= self.target_move_time and self.avoid_step == 0 :
                            self.target_move_time = time.time() + self.duration[1] #waktu maju
                            self.avoid_step = 1
                        #maju
                        if time.time() < self.target_move_time and self.avoid_step == 1:
                            self.get_logger().info('Moving Forward')
                            self.obs_vel.linear.y = self.throtle[2]
                            self.obs_vel.linear.x = 0.0
                        elif time.time() >= self.target_move_time and self.avoid_step == 1 :
                            self.target_move_time = time.time() + self.duration[2] #waktu kembali
                            self.avoid_step = 2
                        #kekiri
                        if time.time() < self.target_move_time and self.avoid_step == 2:
                            self.get_logger().info('Moving Left')
                            self.obs_vel.linear.y = 0.0
                            self.obs_vel.linear.x = self.throtle[3]
                        elif time.time() >= self.target_move_time and self.avoid_step == 2 :
                            self.status_avoid = 0
                            self.avoid_step = 0
                        
                    elif self.status_avoid == 2 : # selesai menghindar
                        self.get_logger().info('Stop')
                        self.obs_vel.linear.y = 0.0
                        self.obs_vel.linear.x = 0.0
                        
                # if self.status_avoid == 3:#gerakan menghindar pipa
                #     if self.imu_yaw <= self.target_yaw + 5 and self.imu_yaw >= self.target_yaw - 5:
                #         self.obs_vel.linear.x = 0.0
                #         self.obs_vel.linear.y = 0.0
                #         self.obs_vel.angular.z = 0.0
                #         self.status_avoid = 4
                # if self.status_avoid == 4: #gerakan memutar kembali ke yaw awal
                #     self.obs_set_point.set_point_yaw = self.imu_yaw + 180
                #     if self.obs_set_point.set_point_yaw > 180 :
                #         self.obs_set_point.set_point_yaw -= 360
                #     elif self.obs_set_point.set_point_yaw < -180 :
                #         self.obs_set_point.set_point_yaw += 360
                #     self.get_logger().info('Success Avoiding Pipe')
                #     self.status_avoid = 0

                img = annotator.result()
                elapsed_time = time.time() - self.fps_start_time
                self.frame_count += 1 
                if elapsed_time > 1:  # Update FPS every second
                    self.fps = self.frame_count / elapsed_time
                    self.fps_start_time = time.time()
                    self.frame_count = 0  # Reset frame counter for next second

                # Display FPS on frame
                fps_text = f"FPS: {self.fps:.1f}"
                cv2.putText(annotated_frame, fps_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                self.status_avoid = 0
                self.avoid_mode = 0
                annotated_frame = self.frame
                # cv2.imshow("camera", self.frame)
                # cv2.waitKey(1)
            cv2.imshow("YOLOv8 Tracking", annotated_frame)
            cv2.waitKey(1)
            self.get_logger().info(f'Status_Avoid {self.status_avoid}')
            self.get_logger().info(f'Step_Avoid {self.avoid_step}')
            self.get_logger().info(f'Avoid Mode {self.avoid_mode}')
        except Exception as e:
            self.get_logger().error(f"Frame is empty: {str(e)}")

    def param_yaml(self):
        try:
            with open(self.yaml_filepath, 'r') as yaml_file:
                data = yaml.safe_load(yaml_file)
            # Extract pid_parameters for serial_node
            obstacle_parameters = data['obstacle_avoidance']

            if obstacle_parameters:
                self.throtle[0] = obstacle_parameters.get('throtle0', 0)
                self.throtle[1] = obstacle_parameters.get('throtle1', 0)
                self.throtle[2] = obstacle_parameters.get('throtle2', 0)
                self.throtle[3] = obstacle_parameters.get('throtle3', 0)
                self.duration[0] = obstacle_parameters.get('timer0', 0)
                self.duration[1] = obstacle_parameters.get('timer1', 0)
                self.duration[2] = obstacle_parameters.get('timer2', 0)
                # self.avoid_mode = obstacle_parameters.get('mode', 0)
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