import rclpy
import sys
import cv2
import os
import numpy as np
import yaml
import logging
from kivy.graphics import *
from datetime import datetime
from kivy.app import App
from kivy.clock import Clock
from kivy.uix.widget import Widget
from kivy.properties import ObjectProperty, NumericProperty, StringProperty
from kivy.lang import Builder
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from kivy.uix.image import Image as KivyImage
from io import BytesIO
from PIL import Image as PILImage
from kivy.graphics.texture import Texture
from timunv2_interfaces.msg import GuiUtilities, SetPoint, SensorData, JoyUtilities, ParamData
from kivy.core.window import Window
from sensor_msgs.msg import Image
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.popup import Popup
from kivy.config import Config
from cv_bridge import CvBridge


cur_dir = "/home/ardiar/ws/timunv2_ws/src/timun_ros2_packages"

CONF =  os.path.join(cur_dir,'timunv2_bringup','config','pidparams.yaml')
SHARE = os.path.join(cur_dir,'timunv2_gui','share')
GUI_KIVY = os.path.join(cur_dir,'timunv2_gui','config','hud.kv')
# CONF =  "/home/tkd/timunv2_ws/src/timunv2_bringup/config/pidparams.yaml"
# SHARE = "/home/tkd/timunv2_ws/src/timunv2_gui/share"
# GUI_KIVY = "/home/tkd/timunv2_ws/src/timunv2_gui/config/hud.kv"
Builder.load_file(GUI_KIVY)

class GuiNode(Node):
    def __init__(self):
        super().__init__('Gui_Node')
        self.roll = 999
        self.pitch = 999
        self.yaw = 999
        self.depth = 999
        self.range = 999
        self.bat_nuc = 999
        self.bat_rob = 999
        self.camF = None
        self.camB = None
        self.arm_hw = False
        self.arm_sw = False
        self.mov_mode = 0
        self.publisher_des      = self.create_publisher(SetPoint,"/gui_set_point",10)
        self.publisher_gui      = self.create_publisher(GuiUtilities,"/gui_utl",10)
        self.publisher_param    = self.create_publisher(ParamData,"/gui_param",10)
        self.subscription       = self.create_subscription(SensorData,'/serial_sensor_data',
                                                    self.listener_callback,10)
        self.sub_camF      = self.create_subscription(Image, '/camera_front', 
                                                    self.listener_camF, 10)
        self.sub_camB      = self.create_subscription(Image, '/camera_bottom', 
                                                    self.listener_camB, 10)
        self.status_sub         = self.create_subscription(JoyUtilities,"/joy_cmd_utl",
                                                    self.status_callback, 10)
        self.subscription
        self.sub_camB
        self.sub_camF
        self.cv_bridge = CvBridge()
    def status_callback(self,msg):
        self.arm_hw     = msg.arm_hw
        self.arm_sw     = msg.arm_sw
        self.mov_mode   = msg.mov_mode
    def listener_callback(self,msg):
            self.roll = msg.imu_yaw
            self.pitch = msg.imu_pitch
            self.yaw = msg.imu_yaw
            self.depth = msg.depth
            self.range = msg.ranges_scan
            # self.bat_nuc = msg.battery_nuc
            # self.bat_rob = msg.battery_robot
    def listener_camF(self,msg):
            self.camF =self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    def listener_camB(self,msg):
            self.camB =self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    def send_var(self):
            return self.roll,self.pitch,self.yaw,self.depth,self.range,self.bat_nuc,self.bat_rob,self.camF,self.camB,self.arm_hw,self.arm_sw,self.mov_mode

class TimunLayout(Widget):
    actual_roll = NumericProperty()
    actual_pitch = NumericProperty()
    actual_yaw = NumericProperty()
    logs = StringProperty()
    frame = ObjectProperty(None)
    def __init__(self,**kwargs):
        super(TimunLayout,self).__init__(**kwargs)
        self.node = GuiNode()
        logging.basicConfig(format='%(asctime)s - %(levelname)s - %(message)s')
        Window.maximize()
        print(str(Window.size))
        with self.ids.pitch.canvas:
            Color(0,0,0,0)
            Rectangle(pos=self.pos, size=Window.size)
        # with self.ids.yaw.canvas:
        #     Color(0,0,0,0)
        #     Rectangle(pos=self.pos, size=Window.size)
        Config.set('kivy', 'exit_on_escape', '0')
        Config.write()

        self.start_timestamp = datetime.now()
        self.count_cam = 0
        self.count_com = 0
        self.actual_roll = 0
        self.actual_pitch = 0
        self.actual_yaw = 0
        self.lumen = 0
        self.logs = ''
        self.ids.throtleB.state ="down"
        self.ids.camF_btn.state ='down'

        self.pitch_x = (Window.size[0]*.95) - (self.ids.roll.size[1]/2)
        self.pitch_y = (Window.size[1]*.825)- (self.ids.roll.size[0]/2)
        self.yaw_x = (Window.size[0]*.5) - 100
        self.yaw_y = (Window.size[1]*.86)
        print(self.pitch_x,self.pitch_y)
        self.ids.cam_main.source = os.path.join(SHARE,'camF.jpg')
        self.ids.roll.source = os.path.join(SHARE,'roll.png')
        self.ids.its.source = os.path.join(SHARE,'1.png')
        self.ids.tekpal.source = os.path.join(SHARE,'2.png')
        self.ids.elektro.source = os.path.join(SHARE,'3.png')
        self.camF = None
        self.camB = None
        self.show = Settings()
        self.msg2 = ParamData()
        # with open(CONF,'r') as f:
        #     self.config = yaml.safe_load(f)
        #     self.msg2.kp_roll           = float(self.config['serial_node']['pid_parameters']['kp_roll'])
        #     self.msg2.ki_roll           = float(self.config['serial_node']['pid_parameters']['ki_roll'])
        #     self.msg2.kd_roll           = float(self.config['serial_node']['pid_parameters']['kd_roll'])
        #     self.msg2.kp_pitch           = float(self.config['serial_node']['pid_parameters']['kp_pitch'])
        #     self.msg2.ki_pitch           = float(self.config['serial_node']['pid_parameters']['ki_pitch'])
        #     self.msg2.kd_pitch           = float(self.config['serial_node']['pid_parameters']['kd_pitch'])
        #     self.msg2.kp_yaw             = float(self.config['serial_node']['pid_parameters']['kp_yaw'])
        #     self.msg2.ki_yaw             = float(self.config['serial_node']['pid_parameters']['ki_yaw'])
        #     self.msg2.kd_yaw             = float(self.config['serial_node']['pid_parameters']['kd_yaw'])
        #     self.msg2.kp_depth           = float(self.config['serial_node']['pid_parameters']['kp_depth'])
        #     self.msg2.ki_depth           = float(self.config['serial_node']['pid_parameters']['ki_depth'])
        #     self.msg2.kd_depth           = float(self.config['serial_node']['pid_parameters']['kd_depth'])
        #     self.msg2.pipe_kp_heading    = float(self.config['pipeline_navigation']['pid_parameters']['kp_heading'])
        #     self.msg2.pipe_ki_heading    = float(self.config['pipeline_navigation']['pid_parameters']['ki_heading'])
        #     self.msg2.pipe_kd_heading    = float(self.config['pipeline_navigation']['pid_parameters']['kd_heading'])
        #     self.msg2.pipe_kp_lateral    = float(self.config['pipeline_navigation']['pid_parameters']['kp_lateral'])
        #     self.msg2.pipe_ki_lateral    = float(self.config['pipeline_navigation']['pid_parameters']['ki_lateral'])
        #     self.msg2.pipe_kd_lateral    = float(self.config['pipeline_navigation']['pid_parameters']['kd_lateral'])
        #     self.msg2.pipe_kp_range      = float(self.config['pipeline_navigation']['pid_parameters']['kp_range'])
        #     self.msg2.pipe_ki_range      = float(self.config['pipeline_navigation']['pid_parameters']['ki_range'])
        #     self.msg2.pipe_kd_range      = float(self.config['pipeline_navigation']['pid_parameters']['kd_range'])
        #     self.msg2.pipe_threshold    = float(self.config['pipeline_navigation']['pid_parameters']['threshold'])
        #     self.msg2.pipe_iou          = float(self.config['pipeline_navigation']['pid_parameters']['iou'])
        #     self.msg2.oa_kp_heading      = float(self.config['obstacle_avoidance']['pid_parameters']['kp_heading'])
        #     self.msg2.oa_ki_heading      = float(self.config['obstacle_avoidance']['pid_parameters']['ki_heading'])
        #     self.msg2.oa_kd_heading      = float(self.config['obstacle_avoidance']['pid_parameters']['kd_heading'])
        #     self.msg2.oa_kp_lateral      = float(self.config['obstacle_avoidance']['pid_parameters']['kp_lateral'])
        #     self.msg2.oa_ki_lateral      = float(self.config['obstacle_avoidance']['pid_parameters']['ki_lateral'])
        #     self.msg2.oa_kd_lateral      = float(self.config['obstacle_avoidance']['pid_parameters']['kd_lateral'])
        #     self.msg2.oa_kp_range        = float(self.config['obstacle_avoidance']['pid_parameters']['kp_range'])
        #     self.msg2.oa_ki_range        = float(self.config['obstacle_avoidance']['pid_parameters']['ki_range'])
        #     self.msg2.oa_kd_range        = float(self.config['obstacle_avoidance']['pid_parameters']['kd_range'])
        #     self.msg2.oa_threshold      = float(self.config['obstacle_avoidance']['pid_parameters']['threshold'])
        #     self.msg2.oa_iou            = float(self.config['obstacle_avoidance']['pid_parameters']['iou'])
        self.setting_popup = Popup(title="Settings", content=self.show, 
                            size_hint=(None,None), size=(800,800))
        logging.getLogger().addHandler(LogUpdateHandler(self))
        Clock.schedule_interval(self.update_cam,0.03)
        Clock.schedule_interval(self.update_sensor,0.01)
        Clock.schedule_interval(self.update_param,1)
        
    def update_param(self,dt):
        with open(CONF,'r') as f:
            self.config = yaml.safe_load(f)
            self.msg2.kp_roll           = float(self.config['serial_node']['pid_parameters']['kp_roll'])
            self.msg2.ki_roll           = float(self.config['serial_node']['pid_parameters']['ki_roll'])
            self.msg2.kd_roll           = float(self.config['serial_node']['pid_parameters']['kd_roll'])
            self.msg2.kp_pitch           = float(self.config['serial_node']['pid_parameters']['kp_pitch'])
            self.msg2.ki_pitch           = float(self.config['serial_node']['pid_parameters']['ki_pitch'])
            self.msg2.kd_pitch           = float(self.config['serial_node']['pid_parameters']['kd_pitch'])
            self.msg2.kp_yaw             = float(self.config['serial_node']['pid_parameters']['kp_yaw'])
            self.msg2.ki_yaw             = float(self.config['serial_node']['pid_parameters']['ki_yaw'])
            self.msg2.kd_yaw             = float(self.config['serial_node']['pid_parameters']['kd_yaw'])
            self.msg2.kp_depth           = float(self.config['serial_node']['pid_parameters']['kp_depth'])
            self.msg2.ki_depth           = float(self.config['serial_node']['pid_parameters']['ki_depth'])
            self.msg2.kd_depth           = float(self.config['serial_node']['pid_parameters']['kd_depth'])
            self.msg2.pipe_kp_heading    = float(self.config['pipeline_navigation']['pid_parameters']['kp_heading'])
            self.msg2.pipe_ki_heading    = float(self.config['pipeline_navigation']['pid_parameters']['ki_heading'])
            self.msg2.pipe_kd_heading    = float(self.config['pipeline_navigation']['pid_parameters']['kd_heading'])
            self.msg2.pipe_kp_lateral    = float(self.config['pipeline_navigation']['pid_parameters']['kp_lateral'])
            self.msg2.pipe_ki_lateral    = float(self.config['pipeline_navigation']['pid_parameters']['ki_lateral'])
            self.msg2.pipe_kd_lateral    = float(self.config['pipeline_navigation']['pid_parameters']['kd_lateral'])
            self.msg2.pipe_kp_range      = float(self.config['pipeline_navigation']['pid_parameters']['kp_range'])
            self.msg2.pipe_ki_range      = float(self.config['pipeline_navigation']['pid_parameters']['ki_range'])
            self.msg2.pipe_kd_range      = float(self.config['pipeline_navigation']['pid_parameters']['kd_range'])
            self.msg2.pipe_threshold    = float(self.config['pipeline_navigation']['pid_parameters']['threshold'])
            self.msg2.pipe_iou          = float(self.config['pipeline_navigation']['pid_parameters']['iou'])
            self.msg2.oa_kp_heading      = float(self.config['obstacle_avoidance']['pid_parameters']['kp_heading'])
            self.msg2.oa_ki_heading      = float(self.config['obstacle_avoidance']['pid_parameters']['ki_heading'])
            self.msg2.oa_kd_heading      = float(self.config['obstacle_avoidance']['pid_parameters']['kd_heading'])
            self.msg2.oa_kp_lateral      = float(self.config['obstacle_avoidance']['pid_parameters']['kp_lateral'])
            self.msg2.oa_ki_lateral      = float(self.config['obstacle_avoidance']['pid_parameters']['ki_lateral'])
            self.msg2.oa_kd_lateral      = float(self.config['obstacle_avoidance']['pid_parameters']['kd_lateral'])
            self.msg2.oa_kp_range        = float(self.config['obstacle_avoidance']['pid_parameters']['kp_range'])
            self.msg2.oa_ki_range        = float(self.config['obstacle_avoidance']['pid_parameters']['ki_range'])
            self.msg2.oa_kd_range        = float(self.config['obstacle_avoidance']['pid_parameters']['kd_range'])
            self.msg2.oa_threshold      = float(self.config['obstacle_avoidance']['pid_parameters']['threshold'])
            self.msg2.oa_iou            = float(self.config['obstacle_avoidance']['pid_parameters']['iou'])
        self.node.publisher_param.publish(self.msg2)

    def update_cam(self,dt):
        rclpy.spin_once(self.node, timeout_sec=1)
        r,p,y,d,ra,bat_n,bat_r,camF,camB,arm_hw,arm_sw,mov_mode = self.node.send_var()
        if(self.ids.camF_btn.state == 'down'):
            cam = camF
        elif(self.ids.camB_btn.state == 'down'):
            cam = camB
        if np.any(cam !=None):
            texture = Texture.create(size=(cam.shape[1], cam.shape[0]), colorfmt='bgr')
            texture.blit_buffer(cam.tobytes(), colorfmt='bgr', bufferfmt='ubyte')
            self.ids.cam_main.texture = texture
        else:
            self.count_cam += 1
            if self.count_cam == 200:
                logging.debug("Trying Connect to Camera...")
                self.count_cam = 0
    def update_sensor(self,dt):
        rclpy.spin_once(self.node, timeout_sec=0.01)
        r,p,y,d,ra,bat_n,bat_r,camF,camB,arm_hw,arm_sw,mov_mode = self.node.send_var()
        if all(x != 999 for x in [r,p,y,d,r]):
            self.actual_roll = round(r,2)
            self.actual_pitch = round(p,2)
            self.actual_yaw = round(y,2)
            self.ids.roll_actual.text =f"{self.actual_roll}'"
            self.ids.pitch_actual.text =f"{self.actual_pitch}'"
            self.ids.yaw_actual.text =f"{self.actual_yaw}'"
            self.ids.depth_actual.text =f"{d} mbar"
            self.ids.range_actual.text =f"{ra} m"
            # self.ids.bat_nuc.text =f"{bat_n}%"
            # self.ids.bat_robot.text =f"{bat_r}%"
        else:
            self.count_com += 1
            if self.count_com == 200:
                logging.debug("Trying Connect to Communication...")
                self.count_com = 0
        if arm_sw == True:
            self.ids.arm.text = "Armed"
        else:
            self.ids.arm.text = "Disarmed"
        if mov_mode == 1:
            self.ids.sub_mode.text = "Stabilize"
        elif mov_mode == 2:
            self.ids.sub_mode.text = "Depth-Hold"
        elif mov_mode == 3:
            self.ids.sub_mode.text = "Depth-Stabilize"
        else:
            self.ids.sub_mode.text = "Manual"
        self.update_gauge()
        self.current_timestamp = datetime.now()
        self.runtime = (self.current_timestamp - self.start_timestamp)
        self.runtime_text = datetime.strptime(str(self.runtime),"%H:%M:%S.%f").strftime("%H:%M:%S")
        self.ids.runtime.text = f"{self.runtime_text}"
        
        # msg = Desired()
        # msg.set_point_roll = check_float(self.ids.roll_des.text)
        # msg.set_point_pitch = check_float(self.ids.pitch_des.text)
        # msg.set_point_yaw = check_float(self.ids.yaw_des.text)
        # msg.set_point_depth = check_float(self.ids.depth_des.text)
        # msg.set_point_range = check_float(self.ids.range_des.text)
        # self.node.publisher_des.publish(msg)
        msg1 = GuiUtilities()
        # msg1.mode_orientation = check_toggle(self.ids.modeO.state)
        # msg1.mode_depth = check_toggle(self.ids.modeD.state)
        msg1.vo_mode = check_toggle(self.ids.VO.state)
        msg1.pipe_mode = check_obj(self.ids.pipeO.state, self.ids.pipeT.state)
        msg1.oa_mode = check_obj(self.ids.oaO.state, self.ids.oaT.state)
        # msg1.record = check_toggle(self.ids.record.state)
        # msg1.calibration = check_toggle(self.ids.calibration.state)
        msg1.throtle_mode = check_throtle(self)
        msg1.lumen = check_lumen(self)
        self.node.publisher_gui.publish(msg1)

    def update_log(self,log_message):
        now = datetime.now().strftime("[%H:%M:%S:%f]")
        formatted_log_message = f'\n{now} {log_message}'
        self.logs += formatted_log_message
        self.ids.log.scroll_y = 0
        
    def update_gauge(self):
        with self.ids.pitch.canvas:
            pos_pitch = [self.pitch_x,self.pitch_y]
            pos_pitch_utl = [self.pitch_x,self.pitch_y+int(self.actual_pitch)]
            StencilPush()
            Color(1,1,1,1)
            # Ellipse(pos=[self.pos[0]+1747,self.pos[1]+763],size=(152,152))
            Ellipse(pos=pos_pitch,size=(152,152))
            StencilUse()
            Rectangle(source=os.path.join(SHARE,'pitch_1.png'),pos=pos_pitch_utl,size=(152,152))
            StencilPop()

    def open_setting(self):
        with open(CONF,'r') as f:
            self.config = yaml.safe_load(f)
            self.show.ids.kproll.text  = f"{self.config['serial_node']['pid_parameters']['kp_roll']}"
            self.show.ids.kiroll.text  = f"{self.config['serial_node']['pid_parameters']['ki_roll']}"
            self.show.ids.kdroll.text  = f"{self.config['serial_node']['pid_parameters']['kd_roll']}"
            self.show.ids.kppitch.text = f"{self.config['serial_node']['pid_parameters']['kp_pitch']}"
            self.show.ids.kipitch.text = f"{self.config['serial_node']['pid_parameters']['ki_pitch']}"
            self.show.ids.kdpitch.text = f"{self.config['serial_node']['pid_parameters']['kd_pitch']}"
            self.show.ids.kpyaw.text   = f"{self.config['serial_node']['pid_parameters']['kp_yaw']}"
            self.show.ids.kiyaw.text   = f"{self.config['serial_node']['pid_parameters']['ki_yaw']}"
            self.show.ids.kdyaw.text   = f"{self.config['serial_node']['pid_parameters']['kd_yaw']}"
            self.show.ids.kpdepth.text = f"{self.config['serial_node']['pid_parameters']['kp_depth']}"
            self.show.ids.kidepth.text = f"{self.config['serial_node']['pid_parameters']['ki_depth']}"
            self.show.ids.kddepth.text = f"{self.config['serial_node']['pid_parameters']['kd_depth']}"
            
            self.show.ids.kppipeH.text = f"{self.config['pipeline_navigation']['pid_parameters']['kp_heading']}"
            self.show.ids.kipipeH.text = f"{self.config['pipeline_navigation']['pid_parameters']['ki_heading']}"
            self.show.ids.kdpipeH.text = f"{self.config['pipeline_navigation']['pid_parameters']['kd_heading']}"
            self.show.ids.kppipeS.text = f"{self.config['pipeline_navigation']['pid_parameters']['kp_lateral']}"
            self.show.ids.kipipeS.text = f"{self.config['pipeline_navigation']['pid_parameters']['ki_lateral']}"
            self.show.ids.kdpipeS.text = f"{self.config['pipeline_navigation']['pid_parameters']['kd_lateral']}"
            self.show.ids.kppipeR.text = f"{self.config['pipeline_navigation']['pid_parameters']['kp_range']}"
            self.show.ids.kipipeR.text = f"{self.config['pipeline_navigation']['pid_parameters']['ki_range']}"
            self.show.ids.kdpipeR.text = f"{self.config['pipeline_navigation']['pid_parameters']['kd_range']}"

            self.show.ids.pipethresh.text = f"{self.config['pipeline_navigation']['pid_parameters']['threshold']}"
            self.show.ids.pipeiou.text    = f"{self.config['pipeline_navigation']['pid_parameters']['iou']}"
            
            self.show.ids.kpoaH.text = f"{self.config['obstacle_avoidance']['pid_parameters']['kp_heading']}"
            self.show.ids.kioaH.text = f"{self.config['obstacle_avoidance']['pid_parameters']['ki_heading']}"
            self.show.ids.kdoaH.text = f"{self.config['obstacle_avoidance']['pid_parameters']['kd_heading']}"
            self.show.ids.kpoaS.text = f"{self.config['obstacle_avoidance']['pid_parameters']['kp_lateral']}"
            self.show.ids.kioaS.text = f"{self.config['obstacle_avoidance']['pid_parameters']['ki_lateral']}"
            self.show.ids.kdoaS.text = f"{self.config['obstacle_avoidance']['pid_parameters']['kd_lateral']}"
            self.show.ids.kpoaR.text = f"{self.config['obstacle_avoidance']['pid_parameters']['kp_range']}"
            self.show.ids.kioaR.text = f"{self.config['obstacle_avoidance']['pid_parameters']['ki_range']}"
            self.show.ids.kdoaR.text = f"{self.config['obstacle_avoidance']['pid_parameters']['kd_range']}"

            self.show.ids.oathresh.text = f"{self.config['obstacle_avoidance']['pid_parameters']['threshold']}"
            self.show.ids.oaiou.text    = f"{self.config['obstacle_avoidance']['pid_parameters']['iou']}"
        self.setting_popup.open()
        
    def throtle_p(self):
        if self.ids.throtleP.state == "normal":
            logging.debug('Update Throtle to Eco Mode')
            self.ids.throtleE.state = "down"
        else:    
            logging.debug('Update Throtle to Power Mode')
            self.ids.throtleE.state = "normal"
            self.ids.throtleB.state = "normal"
    def throtle_e(self):
        if self.ids.throtleE.state == "normal":
            logging.debug('Update Throtle to Balance Mode')
            self.ids.throtleB.state = "down"
        else:
            logging.debug('Update Throtle to Eco Mode')
            self.ids.throtleP.state = "normal"
            self.ids.throtleB.state = "normal"
    def throtle_b(self):
        if self.ids.throtleB.state == "normal":
            logging.debug('Update Throtle to Eco Mode')
            self.ids.throtleE.state = "down"
        else:    
            logging.debug('Update Throtle to Balance Mode')
            self.ids.throtleP.state = "normal"
            self.ids.throtleE.state = "normal"
    def mode_m(self):
        if self.ids.modeM.state == "down":
            self.ids.modeS.state ="normal"
            self.ids.modeD.state ="normal"
            logging.debug('Update Control Mode to Auto')
        else:
            logging.debug('Update Control Mode to Manual')
            self.ids.modeS.state ="normal"
            self.ids.modeD.state ="normal"
            self.ids.modeM.state ="down"
    def mode_m(self):
        if self.ids.modeM.state == "down":
            self.ids.modeS.state ="normal"
            self.ids.modeD.state ="normal"
            logging.debug('Update Control Mode to Manual')
        else:
            logging.debug('Update Control Mode to Manual')
            self.ids.modeS.state ="normal"
            self.ids.modeD.state ="normal"
            self.ids.modeM.state ="down"
    def mode_s(self):
        if self.ids.modeS.state == "down":
            self.ids.modeM.state ="normal"
            self.ids.modeD.state ="normal"
            logging.debug('Update Control Mode to Stabilize')
        else:
            logging.debug('Update Control Mode to Manual')
            self.ids.modeM.state ="down"
            self.ids.modeS.state ="normal"
            self.ids.modeD.state ="normal"
    def mode_d(self):
        if self.ids.modeD.state == "down":
            self.ids.modeM.state ="normal"
            self.ids.modeS.state ="normal"
            logging.debug('Update Control Mode to Depth-Hold')
        else:
            logging.debug('Update Control Mode to Manual')
            self.ids.modeM.state ="down"
            self.ids.modeS.state ="normal"
            self.ids.modeD.state ="normal"
    # def stabilization(self,var,text):
    #     if var == 'down':
    #         logging.debug(f'Update {text} Mode to Stabilization')
    #     else:
    #         logging.debug(f'Update {text} Mode to Manual')
    def objective(self,var,text,text2):
        if var == 'down':
            logging.debug(f'Switch {text} Reference to {text2} Mission')
        else:
            logging.debug(f'Switch {text} Reference to General')
    def camB_btn(self):
        if self.ids.camB_btn.state == 'down':
            self.ids.camF_btn.state = 'normal'
        else:
            self.ids.camF_btn.state = 'down'
            self.ids.camB_btn.state = 'normal'
    def camF_btn(self):
        if self.ids.camF_btn.state == 'down':
            self.ids.camB_btn.state = 'normal'
        else:
            self.ids.camF_btn.state = 'down'
            self.ids.camB_btn.state = 'normal'
    def lumen_plus(self):
        logging.debug(f'Increase Lumen Value')
        self.lumen += 1
    def lumen_min(self):
        logging.debug(f'Decrease Lumen Value')
        self.lumen -= 1
    def update_input(self,text):
        logging.debug(f'Update Value of {text}')
    def update_mode(self,text):
        logging.debug(f'Update mode {text}')


class Settings(FloatLayout):
    def save_conf(self):
        with open(CONF,'r') as f:
            self.config = yaml.safe_load(f)
            self.config['serial_node']['pid_parameters']['kp_roll']             = float(self.ids.kproll.text)
            self.config['serial_node']['pid_parameters']['ki_roll']             = float(self.ids.kiroll.text)
            self.config['serial_node']['pid_parameters']['kd_roll']             = float(self.ids.kdroll.text)
            self.config['serial_node']['pid_parameters']['kp_pitch']            = float(self.ids.kppitch.text)
            self.config['serial_node']['pid_parameters']['ki_pitch']            = float(self.ids.kipitch.text)
            self.config['serial_node']['pid_parameters']['kd_pitch']            = float(self.ids.kdpitch.text)
            self.config['serial_node']['pid_parameters']['kp_yaw']              = float(self.ids.kpyaw.text)
            self.config['serial_node']['pid_parameters']['ki_yaw']              = float(self.ids.kiyaw.text)
            self.config['serial_node']['pid_parameters']['kd_yaw']              = float(self.ids.kdyaw.text)
            self.config['serial_node']['pid_parameters']['kp_depth']            = float(self.ids.kpdepth.text)
            self.config['serial_node']['pid_parameters']['ki_depth']            = float(self.ids.kidepth.text)
            self.config['serial_node']['pid_parameters']['kd_depth']            = float(self.ids.kddepth.text)
            
            self.config['pipeline_navigation']['pid_parameters']['kp_heading']  = float(self.ids.kppipeH.text)
            self.config['pipeline_navigation']['pid_parameters']['ki_heading']  = float(self.ids.kipipeH.text)
            self.config['pipeline_navigation']['pid_parameters']['kd_heading']  = float(self.ids.kdpipeH.text)
            self.config['pipeline_navigation']['pid_parameters']['kp_lateral']  = float(self.ids.kppipeS.text)
            self.config['pipeline_navigation']['pid_parameters']['ki_lateral']  = float(self.ids.kipipeS.text)
            self.config['pipeline_navigation']['pid_parameters']['kd_lateral']  = float(self.ids.kdpipeS.text)
            self.config['pipeline_navigation']['pid_parameters']['kp_range']    = float(self.ids.kppipeR.text)
            self.config['pipeline_navigation']['pid_parameters']['ki_range']    = float(self.ids.kipipeR.text)
            self.config['pipeline_navigation']['pid_parameters']['kd_range']    = float(self.ids.kdpipeR.text)

            # self.config['pipe']['threshold'] = int(self.ids.pipethresh.text)
            # self.config['pipe']['iou'] = int(self.ids.pipeiou.text)

            self.config['obstacle_avoidance']['pid_parameters']['kp_heading']   = float(self.ids.kpoaH.text)
            self.config['obstacle_avoidance']['pid_parameters']['ki_heading']   = float(self.ids.kioaH.text)
            self.config['obstacle_avoidance']['pid_parameters']['kd_heading']   = float(self.ids.kdoaH.text)
            self.config['obstacle_avoidance']['pid_parameters']['kp_lateral']   = float(self.ids.kpoaS.text)
            self.config['obstacle_avoidance']['pid_parameters']['ki_lateral']   = float(self.ids.kioaS.text)
            self.config['obstacle_avoidance']['pid_parameters']['kd_lateral']   = float(self.ids.kdoaS.text)
            self.config['obstacle_avoidance']['pid_parameters']['kp_range']     = float(self.ids.kpoaR.text)
            self.config['obstacle_avoidance']['pid_parameters']['ki_range']     = float(self.ids.kioaR.text)
            self.config['obstacle_avoidance']['pid_parameters']['kd_range']     = float(self.ids.kdoaR.text)

            # self.config['oa']['threshold'] = int(self.ids.oathresh.text)
            # self.config['oa']['iou'] = int(self.ids.oaiou.text)
        with open(CONF,'w') as f:
            yaml.dump(self.config,f)
            logging.debug("Update YAML Configuration")

class LogUpdateHandler(logging.Handler):
    def __init__(self, log_display):
        super(LogUpdateHandler, self).__init__()
        self.log_display = log_display

    def emit(self, record):
        log_message = self.format(record)
        self.log_display.update_log(log_message)
    
class GUI(App):
    def build(self):
        layout = TimunLayout()
        Window.maximize()
        print(str(Window.size))
        return layout

def check_float(var):
    if var == "" or var == "-":
        check = float(0)
    else:    
        check = float(var)
    return check

def check_int(var):
    if var == "" or var == "-":
        check = 0
    else:    
        check = int(var)
    return check

def check_toggle(id_state):
    if id_state == 'normal':
        check = False
    else:
        check = True
    return check

def check_throtle(self):
    if self.ids.throtleP == 'down':
        check = 0
    elif self.ids.throtleE == 'down':
        check = 1
    else:
        check = 2
    return check

def check_obj(state1,state2):
    if state1 == 'down' and state2 == 'down':
        check = 3
    elif state1 == 'normal' and state2 == 'down':
        check = 2
    elif state1 == 'down' and state2 == 'normal':
        check = 1
    else:
        check = 0
    return check
def check_control(self):
    if self.ids.modeM.state == 'down' :
        check = 1
    elif self.ids.modeS.state == 'down':
        check = 2
    elif self.ids.modeD.state == 'down':
        check = 3
    else:
        check = 3
    return check

def check_lumen(self):
    max = 5
    if self.lumen > max:
        self.lumen = max
    elif self.lumen < 0:
        self.lumen = 0
    return self.lumen

def main():
    rclpy.init()
    app = GUI()
    app.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()