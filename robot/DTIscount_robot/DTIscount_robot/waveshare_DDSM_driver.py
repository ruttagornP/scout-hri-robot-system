import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial
import json

class DDSMNode(Node):

    def __init__(self):
        super().__init__('ddsm_node')

        self.left_rpm = 0.0
        self.right_rpm = 0.0

        # =========================
        # SERIAL (แทน HTTP)
        # =========================
        self.port = '/dev/serial0'
        self.baudrate = 115200

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f"✅ Connected to {self.port}")

            # heartbeat (optional)
            #self.set_heartbeat(500)

        except Exception as e:
            self.get_logger().error(f"❌ Serial connect failed: {e}")
            self.ser = None

        # Subscribers
        self.sub_l = self.create_subscription(
            Float64, '/cmd_velocity_left', self.cb_left, 10)

        self.sub_r = self.create_subscription(
            Float64, '/cmd_velocity_right', self.cb_right, 10)

        self.get_logger().info("subcribers created")

        # 20 Hz loop
        self.timer = self.create_timer(0.05, self.send_all)

        self.get_logger().info("DDSM Serial Node Started")

    # =========================
    # CALLBACKS
    # =========================
    def cb_left(self, msg):
        self.left_rpm = msg.data
        #self.get_logger().info(f"LEFT = {msg.data}")

    def cb_right(self, msg):
        self.right_rpm = msg.data
        #self.get_logger().info(f"RIGHT = {msg.data}")

    # =========================
    # HEARTBEAT
    # =========================
    def set_heartbeat(self, time_ms):
        cmd = {"T": 11001, "time": time_ms}
        if self.ser and self.ser.is_open:
            self.ser.write((json.dumps(cmd) + '\n').encode())

    # =========================
    # RPM → CMD
    # =========================
    def rpm_to_cmd(self, rpm):
        scale = 0.3
        return int(rpm * 10.0 * scale)

    # =========================
    # SEND MOTOR
    # =========================
    def send_motor(self, motor_id, cmd):
        payload = {
            "T": 10010,
            "id": motor_id,
            "cmd": cmd,
            "act": 3
        }

        if self.ser and self.ser.is_open:
            self.ser.write((json.dumps(payload) + '\n').encode())

    # =========================
    # MAIN LOOP
    # =========================
    def send_all(self):

        left_cmd = self.rpm_to_cmd(self.left_rpm)
        right_cmd = self.rpm_to_cmd(self.right_rpm)

        # ถ้าล้อกลับด้าน
        right_cmd = -right_cmd

        #left_cmd_send = abs(left_cmd)
        #right_cmd_send = abs(right_cmd)

        # 4 motors
        self.send_motor(1, right_cmd)
        self.send_motor(2, left_cmd)
        self.send_motor(3, right_cmd)
        self.send_motor(4, left_cmd)

        #self.get_logger().info(
        #    f"L={self.left_rpm:.1f}rpm({left_cmd}) | "
        #    f"R={self.right_rpm:.1f}rpm({right_cmd})"
        #)


def main(args=None):
    rclpy.init(args=args)
    node = DDSMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
