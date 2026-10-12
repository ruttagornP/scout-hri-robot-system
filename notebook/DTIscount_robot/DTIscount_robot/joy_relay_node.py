#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8, String

class JoyRelayNode(Node):
    def __init__(self):
        super().__init__("joy_relay_node")

        # ----------------------------
        # 0 = Manual / Joy
        # 1 = Human Follow
        # 2 = Voice
        # ----------------------------
        self.control_mode = 0

        # ----------------------------
        # Publisher
        # ----------------------------
        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        # ----------------------------
        # Subscribe joystick velocity
        # ----------------------------
        self.joy_sub = self.create_subscription(
            Twist,
            "/joy_cmd_vel",
            self.joy_callback,
            10
        )
        
        # ----------------------------
        # Subscribe human follow velocity
        # ----------------------------        
        self.follow_sub = self.create_subscription(
            Twist,
            "/follow_cmd_vel",
            self.follow_callback,
            10
        )

        # ----------------------------
        # Subscribe voice command
        # ----------------------------
        self.voice_sub = self.create_subscription(
            String,
            "/voice_command",
            self.voice_callback,
            10
        )

        # ----------------------------
        # Subscribe control mode
        # ----------------------------
        self.mode_sub = self.create_subscription(
            UInt8,
            "/control_mode",
            self.mode_callback,
            10
        )

        self.get_logger().info("Joy Relay Node Started")

    #######################################################
    def mode_callback(self, msg):
        self.control_mode = msg.data
        if self.control_mode == 0:
            self.get_logger().info("Mode : MANUAL")
        elif self.control_mode == 1:
            self.get_logger().info("Mode : HUMAN FOLLOW")
        elif self.control_mode == 2:
            self.get_logger().info("Mode : VOICE")
        else:
            self.get_logger().warn(
                f"Unknown control mode: {self.control_mode}"
            )

    #######################################################
    def joy_callback(self, msg):
        # Manual Mode
        if self.control_mode == 0:
            self.cmd_pub.publish(msg)
        # Human Following Mode
        else:
            pass
        
    #######################################################
    def follow_callback(self, msg):

        # Human Follow Mode
        if self.control_mode == 1:
            self.cmd_pub.publish(msg)    


    #######################################################
    def voice_callback(self, msg):
        # Voice Mode
        if self.control_mode != 2:
            return

        command = msg.data.strip().upper()

        twist = Twist()

        # ----------------------------
        # Voice command -> Twist
        # ----------------------------
        if command == "FORWARD":
            twist.linear.x = -0.1

        elif command == "BACKWARD":
            twist.linear.x = 0.1

        elif command == "LEFT":
            twist.angular.z = 0.5

        elif command == "RIGHT":
            twist.angular.z = -0.5

        elif command == "STOP":
            # All velocities remain zero
            pass

        else:
            self.get_logger().warn(
                f"Unknown voice command: '{msg.data}'"
            )
            return

        self.cmd_pub.publish(twist)

        self.get_logger().info(
            f"VOICE -> /cmd_vel : {command}"
        )


#############################################################
def main(args=None):
    rclpy.init(args=args)
    node = JoyRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()