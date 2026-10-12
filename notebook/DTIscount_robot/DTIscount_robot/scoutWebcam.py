import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
import subprocess
import shlex

class WebCamGstreamerNode(Node):
    def __init__(self):
        super().__init__('webcam')
        
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('framerate', 30)
        self.declare_parameter('port', 5001)
        
        self.current_width = self.get_parameter('width').value
        self.current_height = self.get_parameter('height').value
        self.current_fps = self.get_parameter('framerate').value
        self.current_port = self.get_parameter('port').value
        
        self.gst_process = None
        
        # Configure media pipeline ก่อนเริ่ม GStreamer
        self._configure_media_pipeline()
        self.start_gstreamer()
        
        self.add_on_set_parameters_callback(self.parameter_callback)

    def _configure_media_pipeline(self):
        """Set format ผ่าน v4l2-ctl และ media-ctl ก่อนเปิด GStreamer"""
        try:
            # Step 1: Set sensor format ผ่าน media-ctl
            media_cmd = (
                f'media-ctl -d /dev/media0 '
                f'--set-v4l2 \'"ov5647 10-0036":0'
                f'[fmt:SGBRG10_1X10/{self.current_width}x{self.current_height}]\''
            )
            self.get_logger().info(f'Configuring media pipeline: {media_cmd}')
            subprocess.run(media_cmd, shell=True, check=True)

            # Step 2: Set v4l2 format ให้ตรงกัน
            v4l2_cmd = (
                f'v4l2-ctl -d /dev/video0 '
                f'--set-fmt-video=width={self.current_width},'
                f'height={self.current_height},pixelformat=BG10'
            )
            self.get_logger().info(f'Configuring v4l2 format: {v4l2_cmd}')
            subprocess.run(v4l2_cmd, shell=True, check=True)

            self.get_logger().info('Media pipeline configured successfully.')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Failed to configure media pipeline: {e}')

    def start_gstreamer(self):
        if self.gst_process is not None and self.gst_process.poll() is None:
            self.get_logger().info('Restarting GStreamer pipeline...')
            self.gst_process.terminate()
            self.gst_process.wait()
        
        self.get_logger().info(
            f'Starting GStreamer: {self.current_width}x{self.current_height} '
            f'@ {self.current_fps}fps on port {self.current_port}'
        )
        
        # ใช้ video/x-bayer แทน video/x-raw สำหรับ OV5647
        gst_cmd = (
            f'gst-launch-1.0 v4l2src device=/dev/video0 '
            f'! video/x-bayer,format=gbrg,'
            f'width={self.current_width},height={self.current_height},'
            f'framerate={self.current_fps}/1 '
            f'! bayer2rgb '
            f'! videoconvert '
            f'! queue max-size-buffers=1 leaky=downstream '
            f'! jpegenc quality=30 '
            f'! multipartmux '
            f'! tcpserversink host=0.0.0.0 port={self.current_port}'
        )
        
        self.get_logger().info(f'GStreamer command: {gst_cmd}')
        
        try:
            self.gst_process = subprocess.Popen(shlex.split(gst_cmd))
            self.get_logger().info(f'Streaming on port {self.current_port}.')
        except Exception as e:
            self.get_logger().error(f'Failed to start GStreamer: {e}')

    def parameter_callback(self, params):
        need_restart = False
        
        for param in params:
            if param.name == 'width' and param.value != self.current_width:
                self.current_width = param.value
                need_restart = True
            elif param.name == 'height' and param.value != self.current_height:
                self.current_height = param.value
                need_restart = True
            elif param.name == 'framerate' and param.value != self.current_fps:
                self.current_fps = param.value
                need_restart = True
            elif param.name == 'port' and param.value != self.current_port:
                self.current_port = param.value
                need_restart = True
        
        if need_restart:
            # ต้อง reconfigure media pipeline ทุกครั้งที่เปลี่ยน resolution
            self._configure_media_pipeline()
            self.start_gstreamer()
            
        return SetParametersResult(successful=True)

    def destroy_node(self):
        if self.gst_process is not None and self.gst_process.poll() is None:
            self.get_logger().info('Terminating GStreamer pipeline...')
            self.gst_process.terminate()
            self.gst_process.wait()
            self.get_logger().info('Camera released.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebCamGstreamerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()