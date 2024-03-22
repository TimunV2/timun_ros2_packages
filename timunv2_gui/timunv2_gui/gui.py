import rclpy
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
from timunv2_interfaces.msg import GuiUtilities, SetPoint, SensorData
from kivy.core.window import Window
from sensor_msgs.msg import Image
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.popup import Popup
from cv_bridge import CvBridge

cur_dir = os.path.dirname(os.path.realpath(__file__))
CONF =  os.path.join(cur_dir,'..','config','params.yaml')
SHARE = os.path.join(cur_dir,'..','share')
GUI_KIVY = os.path.join(cur_dir,'..','config','hud.kv')
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
        self.publisher_des= self.create_publisher(SetPoint,"/desired_gui",10)
        self.publisher_gui= self.create_publisher(GuiUtilities,"/gui_cmd",10)
        self.subscription = self.create_subscription(SensorData,'/rpy',
                                                     self.listener_callback,10)
        self.subscription2 = self.create_subscription(Image, '/webcam/camera', 
                                                self.listener_camF, 10)
        self.subscription
        self.subscription2
        self.cv_bridge = CvBridge()
    def listener_callback(self,msg):
            self.roll = msg.imu_yaw
            self.pitch = msg.imu_pitch
            self.yaw = msg.imu_yaw
            self.depth = msg.depth
            self.range = msg.range_scan
            self.bat_nuc = msg.battery_nuc
            self.bat_rob = msg.battery_robot
    def listener_camF(self,msg):
            self.camF =self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.camB =self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    def listener_camB(self,msg):
            self.camB =self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    def send_var(self):
            return self.roll,self.pitch,self.yaw,self.depth,self.range,self.bat_nuc,self.bat_rob,self.camF,self.camB

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
        with self.ids.pitch.canvas:
            Color(0,0,0,0)
            Rectangle(pos=self.pos, size=Window.size)
        with self.ids.yaw.canvas:
            Color(0,0,0,0)
            Rectangle(pos=self.pos, size=Window.size)
        self.start_timestamp = datetime.now()
        self.count_cam = 0
        self.count_com = 0
        self.actual_roll = 0
        self.actual_pitch = 0
        self.actual_yaw = 0
        self.lumen = 0
        self.logs = ''
        self.ids.modeM.state ="down"
        self.ids.throtleB.state ="down"
        self.ids.camF_btn.state ='down'

        self.pitch_x = 1920*.95 - self.ids.roll.width/2
        self.pitch_y = 1136*.825- self.ids.roll.height/2
        self.ids.cam_main.source = os.path.join(SHARE,'camF.jpg')
        self.ids.roll.source = os.path.join(SHARE,'roll.png')
        self.camF = None
        self.camB = None
        self.show = Settings()
        self.setting_popup = Popup(title="Settings", content=self.show, 
                            size_hint=(None,None), size=(800,800))
        logging.getLogger().addHandler(LogUpdateHandler(self))
        Clock.schedule_interval(self.update,0.05)

    def update(self,dt):
        rclpy.spin_once(self.node, timeout_sec=0.05)
        r,p,y,d,ra,bat_n,bat_r,camF,camB = self.node.send_var()
        
        if(self.ids.camF_btn.state == 'down'):
            cam = camF
        elif(self.ids.camB_btn.state == 'down'):
            cam = camB
        if np.any(cam !=None):
            texture = Texture.create(size=(cam.shape[1], cam.shape[0]), colorfmt='bgr')
            texture.blit_buffer(cam.tobytes(), colorfmt='bgr', bufferfmt='ubyte')
            self.ids.cam_gui.texture = texture
        else:
            self.count_cam += 1
            if self.count_cam == 200:
                logging.debug("Trying Connect to Camera...")
                self.count_cam = 0
        if all(x != 999 for x in [r,p,y,d,ra,bat_n,bat_r]):
            self.actual_roll = r
            self.actual_pitch = p
            self.actual_yaw = y
            self.ids.roll_actual.text =f"{r}'"
            self.ids.pitch_actual.text =f"{p}'"
            self.ids.yaw_actual.text =f"{y}'"
            self.ids.depth_actual.text =f"{d} m"
            self.ids.range_actual.text =f"{ra} m"
            self.ids.bat_nuc.text =f"{bat_n}%"
            self.ids.bat_robot.text =f"{bat_r}%"
        else:
            self.count_com += 1
            if self.count_com == 200:
                logging.debug("Trying Connect to Communication...")
                self.count_com = 0
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
        msg1.mode_auto = check_control(self)
        msg1.mode_pipe = check_obj(self.ids.pipeO.state, self.ids.pipeT.state)
        msg1.mode_oa = check_obj(self.ids.oaO.state, self.ids.oaT.state)
        msg1.mode_map = check_toggle(self.ids.VO.state)
        # msg1.record = check_toggle(self.ids.record.state)
        # msg1.calibration = check_toggle(self.ids.calibration.state)
        msg1.mode_throtle = check_throtle(self)
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
            StencilPush()
            Color(1,1,1,1)
            # Ellipse(pos=[self.pos[0]+1747,self.pos[1]+763],size=(152,152))
            Ellipse(pos=pos_pitch,size=(152,152))
            StencilUse()
            Rectangle(source=os.path.join(SHARE,'pitch_1.png'),pos=pos_pitch,size=(152,152))
            StencilPop()
        with self.ids.yaw.canvas:
            StencilPush()
            Color(1,1,1,.3)
            Rectangle(pos=[self.pos[0]+864,self.pos[1]+874],size=(192,51))
            StencilUse()
            Rectangle(source=os.path.join(SHARE,'yaw.png'),pos=[self.pos[0]+864,self.pos[1]+874],size=(384,51))
            StencilPop()
            
    def open_setting(self):
        with open(CONF,'r') as f:
            self.config = yaml.safe_load(f)
            self.show.ids.kproll.text = f"{self.config['roll']['kp']}"
            self.show.ids.kiroll.text = f"{self.config['roll']['ki']}"
            self.show.ids.kdroll.text = f"{self.config['roll']['kd']}"

            self.show.ids.kppitch.text = f"{self.config['pitch']['kp']}"
            self.show.ids.kipitch.text = f"{self.config['pitch']['ki']}"
            self.show.ids.kdpitch.text = f"{self.config['pitch']['kd']}"

            self.show.ids.kpyaw.text = f"{self.config['yaw']['kp']}"
            self.show.ids.kiyaw.text = f"{self.config['yaw']['ki']}"
            self.show.ids.kdyaw.text = f"{self.config['yaw']['kd']}"
            
            self.show.ids.kpdepth.text = f"{self.config['depth']['kp']}"
            self.show.ids.kidepth.text = f"{self.config['depth']['ki']}"
            self.show.ids.kddepth.text = f"{self.config['depth']['kd']}"
            
            self.show.ids.kppipeH.text = f"{self.config['pipe']['heading']['kp']}"
            self.show.ids.kipipeH.text = f"{self.config['pipe']['heading']['ki']}"
            self.show.ids.kdpipeH.text = f"{self.config['pipe']['heading']['kd']}"
            
            self.show.ids.kppipeS.text = f"{self.config['pipe']['surge']['kp']}"
            self.show.ids.kipipeS.text = f"{self.config['pipe']['surge']['ki']}"
            self.show.ids.kdpipeS.text = f"{self.config['pipe']['surge']['kd']}"
            
            self.show.ids.kppipeR.text = f"{self.config['pipe']['range']['kp']}"
            self.show.ids.kipipeR.text = f"{self.config['pipe']['range']['ki']}"
            self.show.ids.kdpipeR.text = f"{self.config['pipe']['range']['kd']}"

            self.show.ids.pipethresh.text = f"{self.config['pipe']['threshold']}"
            self.show.ids.pipeiou.text = f"{self.config['pipe']['iou']}"
            
            self.show.ids.kpoaH.text = f"{self.config['oa']['heading']['kp']}"
            self.show.ids.kioaH.text = f"{self.config['oa']['heading']['ki']}"
            self.show.ids.kdoaH.text = f"{self.config['oa']['heading']['kd']}"
            
            self.show.ids.kpoaS.text = f"{self.config['oa']['surge']['kp']}"
            self.show.ids.kioaS.text = f"{self.config['oa']['surge']['ki']}"
            self.show.ids.kdoaS.text = f"{self.config['oa']['surge']['kd']}"
            
            self.show.ids.kpoaR.text = f"{self.config['oa']['range']['kp']}"
            self.show.ids.kioaR.text = f"{self.config['oa']['range']['ki']}"
            self.show.ids.kdoaR.text = f"{self.config['oa']['range']['kd']}"

            self.show.ids.oathresh.text = f"{self.config['oa']['threshold']}"
            self.show.ids.oaiou.text = f"{self.config['oa']['iou']}"
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

    def mode_s(self):
        if self.ids.modeS.state == "down":
            self.ids.modeM.state ="normal"
            self.ids.modeD.state ="normal"
            logging.debug('Update Control Mode to Manual')
        else:
            logging.debug('Update Control Mode to Auto')
            self.ids.modeM.state ="down"
            self.ids.modeS.state ="normal"
            self.ids.modeD.state ="normal"
    def mode_d(self):
        if self.ids.modeD.state == "down":
            self.ids.modeM.state ="normal"
            self.ids.modeS.state ="normal"
            logging.debug('Update Control Mode to Manual')
        else:
            logging.debug('Update Control Mode to Auto')
            self.ids.modeM.state ="down"
            self.ids.modeS.state ="normal"
            self.ids.modeD.state ="normal"
    def stabilization(self,var,text):
        if var == 'down':
            logging.debug(f'Update {text} Mode to Stabilization')
        else:
            logging.debug(f'Update {text} Mode to Manual')
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
            self.config['roll']['kp'] = int(self.ids.kproll.text)
            self.config['roll']['ki'] = int(self.ids.kiroll.text)
            self.config['roll']['kd'] = int(self.ids.kdroll.text)

            self.config['pitch']['kp'] = int(self.ids.kppitch.text)
            self.config['pitch']['ki'] = int(self.ids.kipitch.text)
            self.config['pitch']['kd'] = int(self.ids.kdpitch.text)

            self.config['yaw']['kp'] = int(self.ids.kpyaw.text)
            self.config['yaw']['ki'] = int(self.ids.kiyaw.text)
            self.config['yaw']['kd'] = int(self.ids.kdyaw.text)

            self.config['depth']['kp'] = int(self.ids.kpdepth.text)
            self.config['depth']['ki'] = int(self.ids.kidepth.text)
            self.config['depth']['kd'] = int(self.ids.kddepth.text)

            self.config['pipe']['heading']['kp'] = int(self.ids.kppipeH.text)
            self.config['pipe']['heading']['ki'] = int(self.ids.kipipeH.text)
            self.config['pipe']['heading']['kd'] = int(self.ids.kdpipeH.text)

            self.config['pipe']['surge']['kp'] = int(self.ids.kppipeS.text)
            self.config['pipe']['surge']['ki'] = int(self.ids.kipipeS.text)
            self.config['pipe']['surge']['kd'] = int(self.ids.kdpipeS.text)

            self.config['pipe']['range']['kp'] = int(self.ids.kppipeR.text)
            self.config['pipe']['range']['ki'] = int(self.ids.kipipeR.text)
            self.config['pipe']['range']['kd'] = int(self.ids.kdpipeR.text)

            self.config['pipe']['threshold'] = int(self.ids.pipethresh.text)
            self.config['pipe']['iou'] = int(self.ids.pipeiou.text)

            self.config['oa']['heading']['kp'] = int(self.ids.kpoaH.text)
            self.config['oa']['heading']['ki'] = int(self.ids.kioaH.text)
            self.config['oa']['heading']['kd'] = int(self.ids.kdoaH.text)

            self.config['oa']['surge']['kp'] = int(self.ids.kpoaS.text)
            self.config['oa']['surge']['ki'] = int(self.ids.kioaS.text)
            self.config['oa']['surge']['kd'] = int(self.ids.kdoaS.text)

            self.config['oa']['range']['kp'] = int(self.ids.kpoaR.text)
            self.config['oa']['range']['ki'] = int(self.ids.kioaR.text)
            self.config['oa']['range']['kd'] = int(self.ids.kdoaR.text)

            self.config['oa']['threshold'] = int(self.ids.oathresh.text)
            self.config['oa']['iou'] = int(self.ids.oaiou.text)
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