import rclpy
import cv2
import math
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy
from timunv2_interfaces.msg import PingData, SensorData, JoyUtilities, VisdomData, RecordData

class Visdom_Node(Node):
    def __init__(self):
        super().__init__("Visdom_node")
        self.get_logger().info("Visdom_node has been started")
        # self.joy_sub_ = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.camera_sub_ = self.create_subscription(Image, "/camera_bottom", self.image_callback,10)
        # self.ping_sensor_sub_ = self.create_subscription(PingData, "/ping_data", self.ping_callback, 10)
        # self.joy_cmd_utl_sub_ = self.create_subscription(JoyUtilities, "/joy_cmd_utl", self.joy_cmd_utl_callback, 10)
        # self.serial_sensor_data_sub_ = self.create_subscription(SensorData, "/serial_sensor_data", self.serial_sensor_data_callback, 10)
        self.record_sub_ = self.create_subscription(RecordData,"/record_data",self.record_callback,10)

        self.visdom_pub_ = self.create_publisher(VisdomData,"/visdom_value",10)
        self.cv_bridge = CvBridge()
        self.timer_ = self.create_timer(0.03, self.timer_callback)
        self.opr_mode = 1
        self.frame = None
        self.last_frame = None

        self.ping = PingData()
        self.visdom = VisdomData()
        self.imu_yaw = 0
        self.zero_yaw = 0

        #visdom param
        self.FOV = {'H':80,"V":64}
        self.FOVH_RAD = (self.FOV["H"]/2) * 3.14 /180
        self.FOVV_RAD = (self.FOV["V"]/2) * 3.14 /180
        
        self.x_vo = 0
        self.y_vo = 0
        self.z_vo = 0
        self.calc_x_vo = 0
        self.calc_y_vo = 0
        self.visdom.vo_x = 0.0
        self.visdom.vo_y = 0.0
        self.status_visdom = 0
        self.R1_button = 0
        self.R1_button_old = 0
        self.count_vo_record = 0

        self.data_record = RecordData()
    def joy_callback(self,msg):
        self.R1_button = msg.buttons[5]
    def visdom_task(self):
        try:
            if np.all(self.last_frame == None):
                self.last_frame = self.frame
            else:
                if self.opr_mode == 5:
                    height , width, channels = self.frame.shape
                    g_frane = cv2.cvtColor(self.frame,cv2.COLOR_BGR2GRAY)
                    g_lastfrane = cv2.cvtColor(self.last_frame,cv2.COLOR_BGR2GRAY)

                    orb = cv2.ORB_create(nfeatures =240000)
                    bf = cv2.BFMatcher_create(cv2.NORM_HAMMING)

                    kp1,des1 = orb.detectAndCompute(g_frane,None)
                    kp2,des2 = orb.detectAndCompute(g_lastfrane,None)

                    matches = bf.match(des1,des2) 
                    matches = sorted(matches, key= lambda x:x.distance)

                    join_img,x,y,points_now,points_last =self.draw_matches(g_frane, kp1, g_lastfrane, kp2, matches[:100])
                    if len(points_now)>=4 and len(points_last)>=4:
                        points_now = np.float32(points_now[:4])
                        points_last = np.float32(points_last[:4])
                        self.get_logger().info(f"now: {points_now}")
                        self.get_logger().info(f"last: {points_last}")
                        # m ,mask = cv2.findHomography(points_last,points_now)
                        m = cv2.getPerspectiveTransform(points_last,points_now)
                        self.get_logger().info(f"transform: {m}")
                        m /= m[2,2]
                        self.get_logger().info(f"after: {m}")
                        # tx = m[0,2]
                        # ty = m[1,2]
                        t_vector = points_now.mean(axis=0) - points_last.mean(axis=0)
                        tx = t_vector[0]
                        ty = t_vector[1]
                        sx = np.linalg.norm(m[0,:2])
                        sy = np.linalg.norm(m[1,:2])
                        rotation_matrix = m[:2, :2] / np.array([sx, sy])
                        theta = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
                    else:
                        tx = 0
                        ty = 0
                        sx = 0
                        sy = 0
                        theta = 0.0
                    self.get_logger().info(f"translation x: {tx}, y:{ty}")
                    self.get_logger().info(f"x: {x}, y:{y}")
                    self.get_logger().info(f"scale x: {sx}, y:{sy}")
                    self.get_logger().info(f"rotation: {theta}")
                    # r = math.sqrt((x*x)+(y*y))
                    # t = (math.atan2(y,x)*(180/math.pi))-90
                    r = math.sqrt((tx*tx)+(ty*ty))
                    t = (math.atan2(ty,tx)*(180/math.pi))-90
                    # t = rotation_angles
                    if t < 0:
                        tetha_frame = t + 360
                    else:
                        tetha_frame = t

                    vo_tetha = 0 - tetha_frame
                    
                    delta_x = (math.sin(vo_tetha*math.pi/180) * r)
                    delta_y = (math.cos(vo_tetha*math.pi/180) * r)*-1 # *-1 karena mapping +- Y terbalik dari kamera
                    # self.get_logger().info(f"translation_x: {delta_x}")
                    # self.get_logger().info(f"translation_y: {delta_y}")

                    # delta_x = (math.cos(self.imu_yaw) * x) + (math.sin(self.imu_yaw) * y)
                    # delta_y = (math.sin(self.imu_yaw) * x) + (math.cos(self.imu_yaw) * y)

                    # delta_x = (math.cos(self.imu_yaw) * x) - (math.sin(self.imu_yaw) * y)
                    # delta_y = (math.sin(self.imu_yaw) * x) + (math.cos(self.imu_yaw) * y)
                    self.x_vo += delta_x
                    self.y_vo += delta_y
                    # self.x_vo += translation[0]
                    # self.y_vo += translation[1]
                    # self.x_vo += (math.cos(self.imu_yaw)*x) + (math.sin(self.imu_yaw)*y)
                    # self.y_vo += ((math.cos(self.imu_yaw)*y) + (math.sin(self.imu_yaw)*x))*-1
                    # self.x_vo = round(self.x_vo,3)
                    # self.y_vo = round(self.y_vo,3)

                    self.calc_x_vo, self.calc_y_vo = self.count_VO(self.x_vo,self.y_vo,height,width)
                    self.get_logger().info(f"real: {self.count_vo_record}")
                    self.get_logger().info(f"xVO: {self.calc_x_vo}")
                    self.get_logger().info(f"yVO: {self.calc_y_vo}")
                    # self.calc_x_vo = round(self.calc_x_vo,2)
                    # self.calc_y_vo = round(self.calc_y_vo,2)

                    cv2.putText(self.frame, f'VO_x: {round(self.calc_x_vo,2)} m, VO_y: {round(self.calc_y_vo,2)} m', (50,50), cv2.FONT_HERSHEY_SIMPLEX ,  
                    1, (255, 0, 0) , 2, cv2.LINE_AA)
                    cv2.imshow("Visdom", self.frame)
                    cv2.imshow("Join", join_img)
                    cv2.waitKey(1)
                    self.visdom.vo_x = self.calc_x_vo
                    self.visdom.vo_y = self.calc_y_vo
                    
                    self.last_frame = self.frame
                else:
                    self.x_vo = 0
                    self.y_vo = 0
                    cv2.imshow("Visdom",self.frame)
                    cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Frame is empty: {str(e)}")
    
    def count_VO(self,x,y,h,w):
        A_H = 2*self.visdom.vo_z*math.tan(self.FOVH_RAD)
        A_V = 2*self.visdom.vo_z*math.tan(self.FOVV_RAD)
        return (A_H/w)*x, (A_V/h)*y
    def decompose_homography(self,H):
        # Normalize the homography matrix so that H[2, 2] is 1
        H = H / H[2, 2]
        
        # Extract rotation and scaling
        r1 = H[:, 0]
        r2 = H[:, 1]
        t = H[:, 2]

        # Compute scaling factors
        scale_x = np.linalg.norm(r1)
        scale_y = np.linalg.norm(r2)
        
        # Normalize the rotation vectors
        r1 /= scale_x
        r2 /= scale_y
        
        # Ensure a right-handed coordinate system
        r3 = np.cross(r1, r2)
        
        # Rotation matrix
        R = np.stack((r1, r2, r3)).T
        
        # Translation
        translation = t[:2]

        # Scaling
        scaling = (scale_x, scale_y)
        
        return translation, R, scaling

    def draw_matches(self, img1, keypoints1, img2, keypoints2, matches):
        r, c = img1.shape[:2]
        r1, c1 = img2.shape[:2]
        # Create a blank image with the size of the first image + second image
        output_img = np.zeros((max([r, r1]), c+c1, 3), dtype='uint8')
        output_img[:r, :c, :] = np.dstack([img1, img1, img1])
        output_img[:r1, c:c+c1, :] = np.dstack([img2, img2, img2])
        # Go over all of the matching points and extract them
        pixel_shifts = []
        points_img1 =[]
        points_img2 =[]
        offset = 40
        for match in matches:
            img1_idx = match.queryIdx
            img2_idx = match.trainIdx
            (x1, y1) = keypoints1[img1_idx].pt
            (x2, y2) = keypoints2[img2_idx].pt

            shift = (x2 -x1,y2 -y1)
            # print(points_img1)
            if (x2>x1+offset or x2<x1-offset) or (y2>y1+offset or y2<y1-offset) or shift[0]==0 or shift[1]==0:
                continue
            points_img1.append((x1,y1))
            points_img2.append((x2,y2))
            pixel_shifts.append(shift)
            # Draw circles on the keypoints
            cv2.circle(output_img, (int(x1),int(y1)), 4, (0, 255, 255), 1)
            cv2.circle(output_img, (int(x2)+c,int(y2)), 4, (0, 255, 255), 1)

            # Connect the same keypoints
            cv2.line(output_img, (int(x1),int(y1)), (int(x2)+c,int(y2)), (0, 255, 255), 1)
        pixel_shifts_array = np.array(pixel_shifts)
        # print(pixel_shifts_array)
        # mean_x,mean_y = min(pixel_shifts_array[0]),  min(pixel_shifts_array[1])
        if pixel_shifts_array.size>0:
            mean = np.mean(pixel_shifts_array, axis=0)
            mean_x, mean_y = mean
        # print(mean)
        else:
            mean_x = 0
            mean_y = 0
        return output_img,mean_x,mean_y,points_img1,points_img2
    
    def record_callback(self,msg:RecordData):
        self.visdom.vo_z = msg.ranges_scan/1000.0
        self.opr_mode = msg.opr_mode
        self.count_vo_record = msg.count_vo_record
        if self.opr_mode == 5:
            if self.status_visdom == 0:
                self.zero_yaw = msg.imu_yaw
                self.imu_yaw = msg.imu_yaw
                self.status_visdom = 1
                self.get_logger().info("Getting actual yaw as reference")
                self.get_logger().info(f"Starting Visual Odometry Prediction")
                self.get_logger().info(f"Status = {self.status_visdom}")
            else:
                self.imu_yaw = msg.imu_yaw - self.zero_yaw
                self.visdom.count_vo_record = msg.count_vo_record
        else:
            self.visdom.count_vo_record = msg.count_vo_record
            self.imu_yaw = msg.imu_yaw
            self.status_visdom = 0
            self.count_vo_record = 0
            self.visdom.count_vo_record = self.count_vo_record

    def image_callback(self, msg):
        try:
            self.frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {str(e)}")

    def joy_cmd_utl_callback(self, msg:JoyUtilities):
        self.opr_mode = msg.opr_mode

    def ping_callback(self, msg):
        self.visdom.vo_z = msg.ranges_scan/1000.0
        
    def serial_sensor_data_callback(self,msg):
        if self.opr_mode == 5:
            if self.status_visdom == 0:
                self.zero_yaw = msg.imu_yaw
                self.imu_yaw = msg.imu_yaw
                self.status_visdom = 1
                self.get_logger().info("Getting actual yaw as reference")
                self.get_logger().info(f"Starting Visual Odometry Prediction")
                self.get_logger().info(f"Status = {self.status_visdom}")
            else:
                self.imu_yaw = msg.imu_yaw - self.zero_yaw
                if self.R1_button == 1 and self.R1_button_old == 0:
                    self.count_vo_record += 1
                self.R1_button_old = self.R1_button
                self.visdom.count_vo_record = self.count_vo_record

        else:
            self.imu_yaw = msg.imu_yaw
            self.status_visdom = 0
            self.count_vo_record = 0
            self.visdom.count_vo_record = self.count_vo_record

    def timer_callback(self):
        self.visdom_task()
        self.visdom_pub_.publish(self.visdom)

def main(args=None):
    rclpy.init(args=args)
    node=Visdom_Node()
    rclpy.spin(node)
    rclpy.shutdown()