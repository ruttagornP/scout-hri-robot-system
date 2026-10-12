import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import requests
import json

class DDSMNode(Node):

    def __init__(self):
        super().__init__('ddsm_node')

        self.left_rpm = 0.0
        self.right_rpm = 0.0

        # ESP32 Web App
        self.url = "http://192.168.4.1/js"

        # Subscribers
        self.sub_l = self.create_subscription(
            Float64, '/cmd_velocity_left', self.cb_left, 10)
        self.sub_r = self.create_subscription(
            Float64, '/cmd_velocity_right', self.cb_right, 10)

        # ส่งที่ 20 Hz (HTTP ไม่ควรเร็วเกิน)
        self.timer = self.create_timer(0.05, self.send_all)

        self.get_logger().info("DDSM210 Control Node Started")

    # =========================
    # Callbacks
    # =========================
    def cb_left(self, msg):
        self.left_rpm = msg.data

    def cb_right(self, msg):
        self.right_rpm = msg.data

    # =========================
    # RPM → cmd (DDSM210)
    # =========================
    def rpm_to_cmd(self, rpm):
        scale = 0.3
        return int(rpm * 10.0 * scale)   # ⭐ สำคัญ

    # =========================
    # ส่งคำสั่ง motor เดียว
    # =========================
    def send_motor(self, motor_id, cmd):
        payload = {
            "T": 10010,
            "id": motor_id,
            "cmd": cmd,
            "act": 3
        }

        try:
            requests.post(
                self.url,
                data=json.dumps(payload),
                headers={"Content-Type": "application/json"},
                timeout=0.1
            )
        except Exception as e:
            self.get_logger().warn(f"Motor {motor_id} error: {e}")

    # =========================
    # ส่งทั้ง 4 ล้อ
    # =========================
    def send_all(self):

        # แปลงเป็น cmd
        left_cmd = self.rpm_to_cmd(self.left_rpm)
        right_cmd = self.rpm_to_cmd(self.right_rpm)

        # =========================
        # ปรับ direction ถ้าจำเป็น
        # =========================
        right_cmd = -right_cmd   # <-- เปิดถ้าล้อกลับด้าน

        # =========================
        # ส่ง 4 ล้อ
        # =========================
        self.send_motor(1, right_cmd)   # หน้า-ขวา
        self.send_motor(2, left_cmd)   # หน้า-ซ้าย
        self.send_motor(3, right_cmd)  # หลัง-ขวา
        self.send_motor(4, left_cmd)  # หลัง-ซ้าย

        self.get_logger().info(
            f"L={self.left_rpm:.1f}rpm({left_cmd}) | "
            f"R={self.right_rpm:.1f}rpm({right_cmd})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DDSMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()