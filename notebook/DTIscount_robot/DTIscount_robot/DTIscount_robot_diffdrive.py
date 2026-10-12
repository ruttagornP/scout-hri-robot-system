import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math


class CmdVelToDiff(Node):
    def __init__(self, wheel_radius=0.0375, wheel_base=0.16):
        super().__init__('cmdvel_to_diff')

        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base

        self.pub_left = self.create_publisher(Float64, '/cmd_velocity_left', 10)
        self.pub_right = self.create_publisher(Float64, '/cmd_velocity_right', 10)

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info("CmdVelToDiff node ready")

    def cmd_vel_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        # =========================
        # Differential drive
        # =========================
        v_left = linear - (self.wheel_base / 2.0) * angular
        v_right = linear + (self.wheel_base / 2.0) * angular

        # =========================
        # Convert to RPM
        # =========================
        rpm_left = (v_left / (2 * math.pi * self.wheel_radius)) * 60.0
        rpm_right = (v_right / (2 * math.pi * self.wheel_radius)) * 60.0

        # ❌ ห้าม invert ตรงนี้แล้ว
        # ให้ไป invert ที่ EPOS เท่านั้น

        self.pub_left.publish(Float64(data=rpm_left))
        self.pub_right.publish(Float64(data=rpm_right))

        self.get_logger().info(
            f"Linear={linear:.2f} Angular={angular:.2f} | "
            f"L={rpm_left:.1f} R={rpm_right:.1f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToDiff()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()