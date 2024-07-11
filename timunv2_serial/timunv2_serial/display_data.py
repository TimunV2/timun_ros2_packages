import rclpy
from rclpy.node import Node
from timunv2_interfaces.msg import RecordData
import tkinter as tk
from tkinter import ttk

class DataDisplay(Node):
    def __init__(self):
        super().__init__('data_display')
        self.record_sub_ = self.create_subscription(RecordData, '/record_data', self.record_callback, 10)
        
        self.record_data = RecordData()  # Initialize with default RecordData instance
        
        self.root = tk.Tk()
        self.root.title("ROV Data Display")
        
        # Create a canvas with a scrollbar
        self.canvas = tk.Canvas(self.root)
        self.scrollbar = ttk.Scrollbar(self.root, orient="vertical", command=self.canvas.yview)
        self.scrollable_frame = ttk.Frame(self.canvas)

        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.canvas.configure(
                scrollregion=self.canvas.bbox("all")
            )
        )

        self.canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)

        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")
        
        self.joy_cmd_frame = ttk.Frame(self.scrollable_frame)
        self.joy_cmd_frame.grid(row=0, column=0, sticky='nw', padx=10, pady=5)

        self.velocity_frame = ttk.Frame(self.scrollable_frame)
        self.velocity_frame.grid(row=0, column=1, sticky='nw', padx=10, pady=5)

        self.setpoint_frame = ttk.Frame(self.scrollable_frame)
        self.setpoint_frame.grid(row=1, column=0, sticky='nw', padx=10, pady=5)

        self.sensor_data_frame = ttk.Frame(self.scrollable_frame)
        self.sensor_data_frame.grid(row=1, column=1, sticky='nw', padx=10, pady=5)
        
        self.labels = {}

        # Place JoyUtilities fields at the top left
        joy_cmd_fields = [
            'arm_hw', 'arm_sw', 'stabilize', 'depthhold', 'imu_reset', 
            'lumen', 'mov_mode', 'opr_mode', 'data_log', 'max_throtle'
        ]
        for i, field in enumerate(joy_cmd_fields):
            label = ttk.Label(self.joy_cmd_frame, text=f"{field}: N/A", anchor='w')
            label.grid(row=i, column=0, sticky='nw')
            self.labels[field] = label

        # Place velocity fields at the top right
        velocity_fields = [
            'master_linear', 'master_angular', 'pipe_linear', 'pipe_angular'
        ]
        for i, field in enumerate(velocity_fields):
            label = ttk.Label(self.velocity_frame, text=f"{field}: N/A", anchor='w')
            label.grid(row=i, column=0, sticky='nw')
            self.labels[field] = label

        # Place setpoint fields below the joy utilities
        setpoint_fields = [
            'set_point_yaw', 'set_point_pitch', 'set_point_roll', 'set_point_depth'
        ]
        for i, field in enumerate(setpoint_fields):
            label = ttk.Label(self.setpoint_frame, text=f"{field}: N/A", anchor='w')
            label.grid(row=i, column=0, sticky='nw')
            self.labels[field] = label

        # Place sensor data fields below the velocity data
        sensor_data_fields = [
            'imu_yaw', 'imu_pitch', 'imu_roll', 'depth', 'pressure_inside',
            'batter_nuc', 'battery_robot', 'thruster_h_fl', 'thruster_h_fr',
            'thruster_h_bl', 'thruster_h_br', 'thruster_v_fr', 'thruster_v_fl',
            'thruster_v_br', 'thruster_v_bl'
        ]
        for i, field in enumerate(sensor_data_fields):
            label = ttk.Label(self.sensor_data_frame, text=f"{field}: N/A", anchor='w')
            label.grid(row=i, column=0, sticky='nw')
            self.labels[field] = label
        
        self.update_gui()

    def record_callback(self, msg):
        # self.get_logger().info("Received RecordData message")
        self.record_data = msg
        for field in RecordData.__slots__:
            # self.get_logger().info(f"{field}: {getattr(msg, field)}")
            pass
    
    def update_gui(self):
        for field, label in self.labels.items():
            value = getattr(self.record_data, field)
            if hasattr(value, 'x') and hasattr(value, 'y') and hasattr(value, 'z'):
                value_text = f"x: {value.x}, y: {value.y}, z: {value.z}"
            else:
                value_text = value
            label.config(text=f"{field}: {value_text}")
        self.root.after(100, self.update_gui)  # Continue updating the GUI every 100ms

def main(args=None):
    rclpy.init(args=args)
    node = DataDisplay()

    def spin_node():
        try:
            rclpy.spin(node)
        except Exception as e:
            node.get_logger().error(f"Exception in spin: {str(e)}")
        finally:
            rclpy.shutdown()

    import threading
    spin_thread = threading.Thread(target=spin_node, daemon=True)
    spin_thread.start()

    try:
        node.root.mainloop()
    except Exception as e:
        node.get_logger().error(f"Exception in main loop: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
