#import necessary library
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
import time

#import necessary message
from sensor_msgs.msg import Image
from timunv2_interfaces.msg import PipeDetect, JoyUtilities

class PipelineDetect(Node):
    def __init__(self):
        super().__init__("pipeline_detect_node")
        self.get_logger().info("pipeline_detect_node has been started")
        self.camera_sub_ = self.create_subscription(Image, "camera_front", self.image_callback,10)
        self.cmd_utl_sub_ = self.create_subscription(JoyUtilities, "joy_cmd_utl", self.cmd_utl_callback,10)
        self.pipeline_pub_ = self.create_publisher(PipeDetect, "pipeline_value", 10)
        self.timer_ = self.create_timer(0.03, self.timer_callback)
        self.timer_pub_ = self.create_timer(0.01, self.timer_publish)

        self.cv_bridge = CvBridge()

        #sub variable
        self.opr_mode = 1

        #pub variable
        self.pipeline_value = PipeDetect()

        #YOLO Model
        # self.model = YOLO('/home/hasan/ros2_ws/src/timunv2_pipefollowing/timunv2_pipefollowing/pipelinev8n-seg.pt')
        self.model = YOLO('/home/hasan/ros2_ws/src/timunv2_pipefollowing/timunv2_pipefollowing/SelangYOLOV8n.pt')

        #cv variable
        self.frame = None
        self.show_frame = None
        self.masked_frame = None
        self.final_mask = None
        self.fps_start_time = time.time()
        self.fps = 0

    def image_callback(self, msg):
        try:
            self.frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {str(e)}")
    
    def cmd_utl_callback(self, msg:JoyUtilities):
        self.opr_mode = msg.opr_mode
    
    def yolo_inference(self):
        try:
            if self.opr_mode == 0 :
                self.show_frame = self.frame
                
            elif self.opr_mode > 0 :
                results = self.model(self.frame, max_det=2)
                self.show_frame = self.frame

                annotate_frame = results[0].plot()
                cv2.imshow("YOLOv8 Tracking", annotate_frame)
                cv2.waitKey(1)

                # Extract masks of the detected objects
                for r in results:
                    if r.masks is not None:
                        for j, self.masked_frame in enumerate(r.masks.data):
                            self.masked_frame = self.masked_frame.numpy() * 255
                            self.masked_frame = self.masked_frame.astype('uint8')
                
                cv2.imshow("Masked Frame", self.masked_frame)
                cv2.waitKey(1)

                # Find contours and draw largest contour on final_mask
                contours, _ = cv2.findContours(self.masked_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if len(contours) > 0:
                    largest_contour = max(contours, key=cv2.contourArea)
                    self.final_mask = np.zeros_like(self.masked_frame)
                    cv2.drawContours(self.final_mask, [largest_contour], -1, 255, thickness=cv2.FILLED)

                # Process final_mask to find upper and lower detection points, draw lines and circles
                if self.final_mask is not None:
                    frame_width =  self.final_mask.shape[1]
                    frame_height =  self.final_mask.shape[0]

                    # Check the nearest y value of detected pipe from the top of the frame
                    for i in range(frame_height):
                        if np.any(self.final_mask[i, :] > 0):
                            y_upper_detect = i
                            break
                    
                    # Set upper detected point of the pipe
                    line_indices = np.where(self.final_mask[y_upper_detect, :] > 0)
                    x_upper_detect = int(np.mean(line_indices))
                    upper_detect = (x_upper_detect, y_upper_detect)

                    # Set lower detected point of the pipe
                    x_lower_detect = 0
                    line_indices = np.where(self.final_mask[frame_height - 10, :] > 0)
                    if len(line_indices[0]) > 0:
                        x_lower_detect = int(np.mean(line_indices))
                        lower_detect = (x_lower_detect, frame_height - 10)
                    else:
                        lower_detect = (int(frame_width/2), frame_height - 10)

                    # Draw dot and line for the detected pipe
                    cv2.circle(self.show_frame, upper_detect, radius=3, color=(255, 255, 255), thickness=-1)
                    cv2.circle(self.show_frame, lower_detect, radius=3, color=(255, 255, 255), thickness=-1)
                    cv2.line(self.show_frame, lower_detect, upper_detect, color=(255, 0, 0), thickness=2)

                    # Set the center target point for upper and lower frame
                    upper_target = (int(frame_width/2), y_upper_detect)
                    lower_target = (int(frame_width/2), frame_height - 10)

                    cv2.circle(self.show_frame, upper_target, radius=4, color=(0, 0, 255), thickness=-1)
                    cv2.circle(self.show_frame, lower_target, radius=4, color=(0, 0, 255), thickness=-1)

                    cv2.line(self.show_frame, upper_detect, upper_target, color=(0, 0, 255), thickness=1)
                    cv2.line(self.show_frame, lower_detect, lower_target, color=(0, 0, 255), thickness=1)

                    # Offset pixel value that can use as error feedback for controller
                    upper_offset = x_upper_detect - int(frame_width/2)
                    lower_offset = x_lower_detect - int(frame_width/2)
                    self.pipeline_value.upper_offset = upper_offset
                    self.pipeline_value.lower_offset = lower_offset

                    cv2.putText(self.show_frame, str(upper_offset), upper_target, fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255, 255, 255), thickness=1)
                    cv2.putText(self.show_frame, str(lower_offset), lower_target, fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255, 255, 255), thickness=1)

            # Calculate FPS
            fps_end_time = time.time()
            fps_diff_time = fps_end_time - self.fps_start_time
            self.fps = 1 / fps_diff_time
            self.fps_start_time = fps_end_time

            # Display FPS information on the frame
            fps_text = "FPS:{:.2f}".format(self.fps)
            cv2.putText(self.show_frame, fps_text, (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 1)

            # Show the processed frame with annotations
            cv2.imshow("Pipefollowing", self.show_frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Frame is empty: {str(e)}")
    

    def timer_callback(self):
        self.yolo_inference()
        self.pipeline_pub_.publish(self.pipeline_value)
        # pass
    
    def timer_publish(self):
        # self.pipeline_pub_.publish(self.pipeline_value)
        pass

def main(args=None):
    rclpy.init(args=args)
    node = PipelineDetect()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()