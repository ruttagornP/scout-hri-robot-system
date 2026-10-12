import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
import cv2
import socket
import threading
import subprocess

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
        
        self.streaming = False
        self.stream_thread = None
        
        self.start_stream()
        self.add_on_set_parameters_callback(self.parameter_callback)

    def _configure_camera(self):
        """Configure media pipeline ก่อนเปิดกล้องทุกครั้ง"""
        w, h = self.current_width, self.current_height
        subprocess.run(
            f'media-ctl -d /dev/media0 --set-v4l2 \'"ov5647 10-0036":0[fmt:SGBRG10_1X10/{w}x{h}]\'',
            shell=True
        )
        subprocess.run(
            f'v4l2-ctl -d /dev/video0 --set-fmt-video=width={w},height={h},pixelformat=BG10',
            shell=True
        )
        self.get_logger().info(f'Camera configured: {w}x{h}')

    def start_stream(self):
        self.streaming = False
        if self.stream_thread and self.stream_thread.is_alive():
            self.stream_thread.join(timeout=3.0)
        self.streaming = True
        self.stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.stream_thread.start()
        self.get_logger().info(
            f'Streaming started: {self.current_width}x{self.current_height} '
            f'@ {self.current_fps}fps on port {self.current_port}'
        )

    def _handle_client(self, conn, cap):
        try:
            # รับ HTTP request
            request = b''
            while b'\r\n\r\n' not in request:
                chunk = conn.recv(1024)
                if not chunk:
                    return
                request += chunk

            # ส่ง HTTP header
            conn.sendall((
                'HTTP/1.1 200 OK\r\n'
                'Content-Type: multipart/x-mixed-replace; boundary=frame\r\n'
                'Cache-Control: no-cache\r\n'
                'Connection: close\r\n'
                '\r\n'
            ).encode())

            # ส่ง frame ต่อเนื่อง
            while self.streaming:
                ret, frame = cap.read()
                if not ret:
                    self.get_logger().warn('Failed to read frame')
                    break
                _, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 30])
                jpg_bytes = jpg.tobytes()
                part = (
                    f'--frame\r\n'
                    f'Content-Type: image/jpeg\r\n'
                    f'Content-Length: {len(jpg_bytes)}\r\n\r\n'
                ).encode() + jpg_bytes + b'\r\n'
                conn.sendall(part)

        except (BrokenPipeError, ConnectionResetError, OSError):
            self.get_logger().info('Client disconnected')
        finally:
            conn.close()

    def _stream_loop(self):
        # Configure ก่อนทุกครั้ง
        self._configure_camera()

        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.current_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.current_height)
        cap.set(cv2.CAP_PROP_FPS, self.current_fps)

        if not cap.isOpened():
            self.get_logger().error('Cannot open camera!')
            return

        # ทดสอบอ่าน frame แรก
        ret, _ = cap.read()
        if not ret:
            self.get_logger().error('Camera opened but cannot read frame!')
            cap.release()
            return

        self.get_logger().info(
            f'Camera ready: '
            f'{int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
            f'{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}'
        )

        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', self.current_port))
        server.listen(5)
        server.settimeout(1.0)
        self.get_logger().info(f'Waiting for client on port {self.current_port}...')

        while self.streaming:
            try:
                conn, addr = server.accept()
                self.get_logger().info(f'Client connected: {addr}')
                threading.Thread(
                    target=self._handle_client,
                    args=(conn, cap),
                    daemon=True
                ).start()
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f'Server error: {e}')
                break

        cap.release()
        server.close()
        self.get_logger().info('Stream stopped')

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
            self.start_stream()
        return SetParametersResult(successful=True)

    def destroy_node(self):
        self.streaming = False
        if self.stream_thread:
            self.stream_thread.join(timeout=3.0)
        self.get_logger().info('Camera node stopped.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebCamGstreamerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
