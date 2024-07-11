import rclpy
from rclpy.node import Node
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

class CameraStreamer(Node):
    def __init__(self):
        super().__init__('gst_streamer')
        Gst.init(None)

        # Pipeline for Camera 1
        self.pipeline1 = Gst.parse_launch(
            'v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay ! udpsink host=192.168.2.1 port=5000'
        )
        
        # Pipeline for Camera 2
        self.pipeline2 = Gst.parse_launch(
            'v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay ! udpsink host=192.168.2.1 port=5001'
        )

        self.pipeline1.set_state(Gst.State.PLAYING)
        self.pipeline2.set_state(Gst.State.PLAYING)
        
        self.get_logger().info('GStreamer pipelines are running.')

def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down pipelines.')
    finally:
        node.pipeline1.set_state(Gst.State.NULL)
        node.pipeline2.set_state(Gst.State.NULL)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
