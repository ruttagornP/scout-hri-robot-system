#!/usr/bin/env python3

import time
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String


class AdaptiveControllerV2(Node):

    def __init__(self):
        super().__init__("adaptive_controller_v2")

        # ============================================================
        # Controller configuration
        # ============================================================

        self.controller_period = 1.0

        # ------------------------------------------------------------
        # Video profiles
        # ------------------------------------------------------------

        self.profiles = {
            "HIGH": {
                "mode": "single",
                "width": 640,
                "height": 480,
                "fps": 15,
            },

            "MEDIUM": {
                "mode": "single",
                "width": 320,
                "height": 240,
                "fps": 10,
            },

            "LOW": {
                "mode": "multicast",
                "width": 320,
                "height": 240,
                "fps": 5,
            },
        }

        # ============================================================
        # Current configuration
        # ============================================================

        self.current_mode = "single"

        self.current_width = 640
        self.current_height = 480
        self.current_fps = 15

        self.current_profile = "HIGH"

        # ============================================================
        # Desired configuration
        # ============================================================

        self.desired_profile = "HIGH"

        self.desired_mode = "single"
        self.desired_width = 640
        self.desired_height = 480
        self.desired_fps = 15

        # ============================================================
        # Notebook / network QoS metrics
        # ============================================================

        self.metrics_received = False

        self.actual_fps = 0.0
        self.video_mbps = 0.0
        self.rtp_packet_rate = 0.0

        self.rtp_packets = 0.0
        self.rtp_bytes = 0.0
        self.rtp_lost_packets = 0.0

        self.rtp_loss_percent = 0.0

        self.rtp_duplicate_packets = 0.0
        self.rtp_out_of_order_packets = 0.0

        self.rtp_jitter_ms = 0.0

        self.width = 0
        self.height = 0
        self.target_fps = 0

        # ============================================================
        # Raspberry Pi / receiver metrics
        # ============================================================

        self.pi_metrics_received = False

        self.pi_decode_fps = 0.0
        self.pi_frame_age_ms = 0.0
        self.pi_cpu_percent = 0.0
        self.pi_video_active = 0.0

        # ============================================================
        # Transport mode received from GUI
        # ============================================================

        self.transport_mode_received = False
        self.gui_transport_mode = "single"

        # ============================================================
        # QoS score
        # ============================================================

        self.fps_score = 0.0
        self.loss_score = 0.0
        self.jitter_score = 0.0
        self.frame_age_score = 0.0
        self.cpu_score = 0.0

        self.qos_score = 0.0

        # ============================================================
        # Network state
        # ============================================================

        self.network_state = "UNKNOWN"

        # ============================================================
        # Hysteresis counters
        # ============================================================

        self.high_count = 0
        self.medium_count = 0
        self.low_count = 0
        
        # ------------------------------------------------------------
        # Explicit adaptive FSM state
        # ------------------------------------------------------------
        # This is the profile state used by the adaptive algorithm.
        # In DRY-RUN mode this state is allowed to change internally.
        # In LIVE mode this state will follow the real configuration.
        self.adaptive_profile = "HIGH"

        # ------------------------------------------------------------
        # Number of consecutive samples required
        # ------------------------------------------------------------

        self.high_to_medium_count = 3
        self.medium_to_low_count = 3

        self.low_to_medium_count = 5
        self.medium_to_high_count = 5

        # ============================================================
        # Score thresholds
        # ============================================================

        # Degradation
        self.high_to_medium_score = 65.0
        self.medium_to_low_score = 35.0

        # Recovery
        self.low_to_medium_score = 55.0
        self.medium_to_high_score = 75.0

        # ============================================================
        # Decision publishing
        # ============================================================

        self.last_published_profile = None

        # ============================================================
        # ROS subscriptions
        # ============================================================

        self.metrics_subscription = self.create_subscription(
            Float64MultiArray,
            "/qos/metrics",
            self.metrics_callback,
            10
        )

        self.pi_metrics_subscription = self.create_subscription(
            Float64MultiArray,
            "/qos/pi_metrics",
            self.pi_metrics_callback,
            10
        )

        self.transport_subscription = self.create_subscription(
            String,
            "/qos/transport_mode",
            self.transport_mode_callback,
            10
        )

        # ============================================================
        # ROS publishers
        # ============================================================

        self.decision_publisher = self.create_publisher(
            String,
            "/adaptive/decision",
            10
        )

        self.state_publisher = self.create_publisher(
            String,
            "/adaptive/state",
            10
        )

        self.score_publisher = self.create_publisher(
            Float64MultiArray,
            "/adaptive/qos_score",
            10
        )

        # ============================================================
        # Controller timer
        # ============================================================

        self.timer = self.create_timer(
            self.controller_period,
            self.controller_loop
        )

        # ============================================================
        # Start time
        # ============================================================

        self.start_time = time.monotonic()

        # ============================================================
        # Startup messages
        # ============================================================

        self.get_logger().info(
            "=============================================="
        )

        self.get_logger().info(
            "Adaptive Controller V2 Started"
        )

        self.get_logger().info(
            "Mode: DRY-RUN"
        )

        self.get_logger().info(
            "Profiles:"
        )

        self.get_logger().info(
            "  HIGH   = SINGLE 640x480 @ 15 FPS"
        )

        self.get_logger().info(
            "  MEDIUM = SINGLE 320x240 @ 10 FPS"
        )

        self.get_logger().info(
            "  LOW    = MULTICAST 320x240 @ 5 FPS"
        )

        self.get_logger().info(
            "QoS Score = FPS + Loss + Jitter + FrameAge + CPU"
        )

        self.get_logger().info(
            "=============================================="
        )

    # ================================================================
    # /qos/metrics callback
    # ================================================================

    def metrics_callback(self, msg):

        data = list(msg.data)

        if len(data) < 12:

            self.get_logger().warning(
                f"Invalid /qos/metrics: "
                f"expected >= 12 values, got {len(data)}"
            )

            return

        # ------------------------------------------------------------
        # Decode metrics
        #
        # 0  = actual FPS
        # 1  = video Mbps
        # 2  = RTP packet rate
        # 3  = RTP packets
        # 4  = RTP bytes
        # 5  = RTP lost packets
        # 6  = RTP duplicate packets
        # 7  = RTP out-of-order packets
        # 8  = RTP jitter ms
        # 9  = width
        # 10 = height
        # 11 = target FPS
        # ------------------------------------------------------------

        self.actual_fps = float(data[0])

        self.video_mbps = float(data[1])

        self.rtp_packet_rate = float(data[2])

        self.rtp_packets = float(data[3])

        self.rtp_bytes = float(data[4])

        self.rtp_lost_packets = float(data[5])

        self.rtp_duplicate_packets = float(data[6])

        self.rtp_out_of_order_packets = float(data[7])

        self.rtp_jitter_ms = float(data[8])

        self.width = int(data[9])

        self.height = int(data[10])

        self.target_fps = int(data[11])

        # ------------------------------------------------------------
        # Calculate packet loss
        # ------------------------------------------------------------

        total_packets = (
            self.rtp_packets +
            self.rtp_lost_packets
        )

        if total_packets > 0:

            self.rtp_loss_percent = (
                self.rtp_lost_packets /
                total_packets
            ) * 100.0

        else:

            self.rtp_loss_percent = 0.0

        # ------------------------------------------------------------
        # Update current video configuration
        # ------------------------------------------------------------

        if self.width > 0:
            self.current_width = self.width

        if self.height > 0:
            self.current_height = self.height

        if self.target_fps > 0:
            self.current_fps = self.target_fps

        # ------------------------------------------------------------
        # Detect profile from video parameters
        # ------------------------------------------------------------

        self.detect_current_profile()

        self.metrics_received = True

    # ================================================================
    # /qos/pi_metrics callback
    # ================================================================

    def pi_metrics_callback(self, msg):

        data = list(msg.data)

        if len(data) < 4:

            self.get_logger().warning(
                f"Invalid /qos/pi_metrics: "
                f"expected >= 4 values, got {len(data)}"
            )

            return

        # ------------------------------------------------------------
        # Pi metrics
        #
        # 0 = decode FPS
        # 1 = frame age ms
        # 2 = CPU percent
        # 3 = video active
        # ------------------------------------------------------------

        self.pi_decode_fps = float(data[0])

        self.pi_frame_age_ms = float(data[1])

        self.pi_cpu_percent = float(data[2])

        self.pi_video_active = float(data[3])

        self.pi_metrics_received = True

    # ================================================================
    # /qos/transport_mode callback
    # ================================================================

    def transport_mode_callback(self, msg):

        mode = msg.data.strip().lower()

        if mode not in ("single", "multicast"):

            self.get_logger().warning(
                f"Unknown transport mode: {mode}"
            )

            return

        self.gui_transport_mode = mode

        self.transport_mode_received = True

        self.current_mode = mode

    # ================================================================
    # Detect current profile
    # ================================================================

    def detect_current_profile(self):

        # ------------------------------------------------------------
        # HIGH
        # ------------------------------------------------------------

        if (
            self.current_mode == "single"
            and self.current_width >= 640
            and self.current_height >= 480
            and self.current_fps >= 15
        ):

            self.current_profile = "HIGH"

            return

        # ------------------------------------------------------------
        # LOW
        # ------------------------------------------------------------

        if (
            self.current_mode == "multicast"
            and self.current_width <= 320
            and self.current_height <= 240
            and self.current_fps <= 5
        ):

            self.current_profile = "LOW"

            return

        # ------------------------------------------------------------
        # MEDIUM
        # ------------------------------------------------------------

        if (
            self.current_mode == "single"
            and self.current_width <= 320
            and self.current_height <= 240
            and self.current_fps <= 10
        ):

            self.current_profile = "MEDIUM"

            return

        # ------------------------------------------------------------
        # Unknown intermediate configuration
        # ------------------------------------------------------------

        self.current_profile = "UNKNOWN"

    # ================================================================
    # Utility clamp
    # ================================================================

    def clamp(self, value, minimum, maximum):

        return max(
            minimum,
            min(maximum, value)
        )

    # ================================================================
    # Calculate FPS score
    # ================================================================

    def calculate_fps_score(self):

        target = self.target_fps

        if target <= 0:

            target = self.current_fps

        if target <= 0:

            return 0.0

        ratio = self.actual_fps / float(target)

        score = ratio * 100.0

        return self.clamp(
            score,
            0.0,
            100.0
        )

    # ================================================================
    # Calculate packet loss score
    # ================================================================

    def calculate_loss_score(self):

        loss = self.rtp_loss_percent

        # ------------------------------------------------------------
        # 0.5% or lower = excellent
        # 10% or higher = unacceptable
        # ------------------------------------------------------------

        excellent = 0.5
        unacceptable = 10.0

        if loss <= excellent:

            return 100.0

        if loss >= unacceptable:

            return 0.0

        score = (
            (unacceptable - loss)
            /
            (unacceptable - excellent)
        ) * 100.0

        return self.clamp(
            score,
            0.0,
            100.0
        )

    # ================================================================
    # Calculate jitter score
    # ================================================================

    def calculate_jitter_score(self):

        jitter = self.rtp_jitter_ms

        # ------------------------------------------------------------
        # 5 ms or lower = excellent
        # 50 ms or higher = poor
        # ------------------------------------------------------------

        excellent = 5.0
        unacceptable = 50.0

        if jitter <= excellent:

            return 100.0

        if jitter >= unacceptable:

            return 0.0

        score = (
            (unacceptable - jitter)
            /
            (unacceptable - excellent)
        ) * 100.0

        return self.clamp(
            score,
            0.0,
            100.0
        )

    # ================================================================
    # Calculate frame age score
    # ================================================================

    def calculate_frame_age_score(self):

        # ------------------------------------------------------------
        # If Pi metrics are not available yet
        # ------------------------------------------------------------

        if not self.pi_metrics_received:

            return 50.0

        age = self.pi_frame_age_ms

        # ------------------------------------------------------------
        # 100 ms or lower = excellent
        # 500 ms or higher = poor
        # ------------------------------------------------------------

        excellent = 100.0
        unacceptable = 500.0

        if age <= excellent:

            return 100.0

        if age >= unacceptable:

            return 0.0

        score = (
            (unacceptable - age)
            /
            (unacceptable - excellent)
        ) * 100.0

        return self.clamp(
            score,
            0.0,
            100.0
        )

    # ================================================================
    # Calculate CPU score
    # ================================================================

    def calculate_cpu_score(self):

        # ------------------------------------------------------------
        # If Pi metrics unavailable
        # ------------------------------------------------------------

        if not self.pi_metrics_received:

            return 50.0

        cpu = self.pi_cpu_percent

        # ------------------------------------------------------------
        # <= 60% = excellent
        # >= 95% = poor
        # ------------------------------------------------------------

        excellent = 60.0
        unacceptable = 95.0

        if cpu <= excellent:

            return 100.0

        if cpu >= unacceptable:

            return 0.0

        score = (
            (unacceptable - cpu)
            /
            (unacceptable - excellent)
        ) * 100.0

        return self.clamp(
            score,
            0.0,
            100.0
        )

    # ================================================================
    # Calculate overall QoS score
    # ================================================================

    def calculate_qos_score(self):

        self.fps_score = (
            self.calculate_fps_score()
        )

        self.loss_score = (
            self.calculate_loss_score()
        )

        self.jitter_score = (
            self.calculate_jitter_score()
        )

        self.frame_age_score = (
            self.calculate_frame_age_score()
        )

        self.cpu_score = (
            self.calculate_cpu_score()
        )

        # ------------------------------------------------------------
        # Weighted QoS score
        #
        # FPS       = 25%
        # Loss      = 25%
        # Jitter    = 15%
        # Frame age = 20%
        # CPU       = 15%
        # ------------------------------------------------------------

        self.qos_score = (

            0.25 * self.fps_score

            +

            0.25 * self.loss_score

            +

            0.15 * self.jitter_score

            +

            0.20 * self.frame_age_score

            +

            0.15 * self.cpu_score
        )

        self.qos_score = self.clamp(
            self.qos_score,
            0.0,
            100.0
        )

        return self.qos_score

    # ================================================================
    # Evaluate network state
    # ================================================================

    def evaluate_network(self):

        score = self.qos_score

        # ------------------------------------------------------------
        # Additional hard safety conditions
        #
        # Score alone should not hide severe conditions.
        # ------------------------------------------------------------

        severe_loss = (
            self.rtp_loss_percent >= 10.0
        )

        severe_jitter = (
            self.rtp_jitter_ms >= 50.0
        )

        severe_frame_age = (
            self.pi_frame_age_ms >= 500.0
            and self.pi_metrics_received
        )

        very_low_fps = (
            self.actual_fps < 5.0
        )

        if (
            severe_loss
            or severe_jitter
            or severe_frame_age
            or very_low_fps
        ):

            return "BAD"

        # ------------------------------------------------------------
        # Score based state
        # ------------------------------------------------------------

        if score < 35.0:

            return "BAD"

        if score < 65.0:

            return "WARNING"

        return "GOOD"

    # ================================================================
    # Reset counters
    # ================================================================

    def reset_counters(self):

        self.high_count = 0
        self.medium_count = 0
        self.low_count = 0

    # ================================================================
    # Set desired profile
    # ================================================================

    def set_desired_profile(self, profile):

        if profile not in self.profiles:

            self.get_logger().error(
                f"Invalid profile: {profile}"
            )

            return

        config = self.profiles[profile]

        self.desired_profile = profile

        self.desired_mode = config["mode"]

        self.desired_width = config["width"]

        self.desired_height = config["height"]

        self.desired_fps = config["fps"]

    # ================================================================
    # Adaptive decision - Explicit Hysteresis FSM
    # ================================================================

    def make_decision(self):

        score = self.qos_score

        # ------------------------------------------------------------
        # Current adaptive state
        # ------------------------------------------------------------

        current = self.adaptive_profile

        # ============================================================
        # HIGH STATE
        # ============================================================

        if current == "HIGH":

            # HIGH -> MEDIUM degradation counter
            self.high_count = 0
            self.low_count = 0

            if score < self.high_to_medium_score:

                self.medium_count += 1

                self.get_logger().info(
                    f"[HYSTERESIS] HIGH -> MEDIUM "
                    f"candidate {self.medium_count}/"
                    f"{self.high_to_medium_count} "
                    f"(score={score:.2f})"
                )

            else:

                self.medium_count = 0

            # --------------------------------------------------------
            # Confirm HIGH -> MEDIUM
            # --------------------------------------------------------

            if (
                self.medium_count
                >= self.high_to_medium_count
            ):

                self.adaptive_profile = "MEDIUM"

                self.medium_count = 0

                self.get_logger().warn(
                    "[ADAPTIVE FSM] HIGH -> MEDIUM"
                )

            # Desired profile follows adaptive state
            self.set_desired_profile(
                self.adaptive_profile
            )

            return

        # ============================================================
        # MEDIUM STATE
        # ============================================================

        if current == "MEDIUM":

            # --------------------------------------------------------
            # MEDIUM -> LOW
            # --------------------------------------------------------

            if score < self.medium_to_low_score:

                self.low_count += 1

                self.get_logger().info(
                    f"[HYSTERESIS] MEDIUM -> LOW "
                    f"candidate {self.low_count}/"
                    f"{self.medium_to_low_count} "
                    f"(score={score:.2f})"
                )

            else:

                self.low_count = 0

            # --------------------------------------------------------
            # MEDIUM -> HIGH
            # --------------------------------------------------------

            if score > self.medium_to_high_score:

                self.high_count += 1

                self.get_logger().info(
                    f"[HYSTERESIS] MEDIUM -> HIGH "
                    f"candidate {self.high_count}/"
                    f"{self.medium_to_high_count} "
                    f"(score={score:.2f})"
                )

            else:

                self.high_count = 0

            # --------------------------------------------------------
            # Confirm MEDIUM -> LOW
            # --------------------------------------------------------

            if (
                self.low_count
                >= self.medium_to_low_count
            ):

                self.adaptive_profile = "LOW"

                self.low_count = 0
                self.high_count = 0

                self.get_logger().warn(
                    "[ADAPTIVE FSM] MEDIUM -> LOW"
                )

                self.set_desired_profile("LOW")

                return

            # --------------------------------------------------------
            # Confirm MEDIUM -> HIGH
            # --------------------------------------------------------

            if (
                self.high_count
                >= self.medium_to_high_count
            ):

                self.adaptive_profile = "HIGH"

                self.high_count = 0
                self.low_count = 0

                self.get_logger().info(
                    "[ADAPTIVE FSM] MEDIUM -> HIGH"
                )

                self.set_desired_profile("HIGH")

                return

            # --------------------------------------------------------
            # Stay MEDIUM
            # --------------------------------------------------------

            self.set_desired_profile("MEDIUM")

            return

        # ============================================================
        # LOW STATE
        # ============================================================

        if current == "LOW":

            self.high_count = 0
            self.low_count = 0

            # --------------------------------------------------------
            # LOW -> MEDIUM
            # --------------------------------------------------------

            if score > self.low_to_medium_score:

                self.medium_count += 1

                self.get_logger().info(
                    f"[HYSTERESIS] LOW -> MEDIUM "
                    f"candidate {self.medium_count}/"
                    f"{self.low_to_medium_count} "
                    f"(score={score:.2f})"
                )

            else:

                self.medium_count = 0

            # --------------------------------------------------------
            # Confirm LOW -> MEDIUM
            # --------------------------------------------------------

            if (
                self.medium_count
                >= self.low_to_medium_count
            ):

                self.adaptive_profile = "MEDIUM"

                self.medium_count = 0

                self.get_logger().info(
                    "[ADAPTIVE FSM] LOW -> MEDIUM"
                )

                self.set_desired_profile("MEDIUM")

                return

            # --------------------------------------------------------
            # Stay LOW
            # --------------------------------------------------------

            self.set_desired_profile("LOW")

            return

        # ============================================================
        # UNKNOWN STATE
        # ============================================================

        self.get_logger().warning(
            f"[ADAPTIVE FSM] Unknown state: {current}. "
            f"Resetting to MEDIUM."
        )

        self.adaptive_profile = "MEDIUM"

        self.reset_counters()

        self.set_desired_profile("MEDIUM")

    # ================================================================
    # Check decision change
    # ================================================================

    def check_decision_change(self):

        mode_changed = (
            self.desired_mode
            != self.current_mode
        )

        resolution_changed = (

            self.desired_width
            != self.current_width

            or

            self.desired_height
            != self.current_height
        )

        fps_changed = (
            self.desired_fps
            != self.current_fps
        )

        profile_changed = (
            self.desired_profile
            != self.last_published_profile
        )

        return (
            mode_changed
            or resolution_changed
            or fps_changed
            or profile_changed
        )

    # ================================================================
    # Publish decision
    # ================================================================

    def publish_decision(self):

        msg = String()

        msg.data = (
            f"profile={self.desired_profile},"
            f"mode={self.desired_mode},"
            f"resolution="
            f"{self.desired_width}x"
            f"{self.desired_height},"
            f"fps={self.desired_fps},"
            f"score={self.qos_score:.2f},"
            f"state={self.network_state}"
        )

        self.decision_publisher.publish(msg)

        self.last_published_profile = (
            self.desired_profile
        )

    # ================================================================
    # Publish controller state
    # ================================================================

    def publish_state(self):

        msg = String()

        msg.data = (

            f"state={self.network_state},"

            f"score={self.qos_score:.2f},"

            f"current_profile="
            f"{self.current_profile},"

            f"current_mode="
            f"{self.current_mode},"

            f"current="
            f"{self.current_width}x"
            f"{self.current_height}"
            f"@{self.current_fps},"

            f"desired_profile="
            f"{self.desired_profile},"

            f"desired_mode="
            f"{self.desired_mode},"

            f"desired="
            f"{self.desired_width}x"
            f"{self.desired_height}"
            f"@{self.desired_fps}"
        )

        self.state_publisher.publish(msg)

    # ================================================================
    # Publish QoS score
    # ================================================================

    def publish_score(self):

        msg = Float64MultiArray()

        # ------------------------------------------------------------
        # [0] overall score
        # [1] FPS score
        # [2] loss score
        # [3] jitter score
        # [4] frame age score
        # [5] CPU score
        # ------------------------------------------------------------

        msg.data = [

            float(self.qos_score),

            float(self.fps_score),

            float(self.loss_score),

            float(self.jitter_score),

            float(self.frame_age_score),

            float(self.cpu_score),
        ]

        self.score_publisher.publish(msg)

    # ================================================================
    # Print detailed decision information
    # ================================================================

    def print_decision(self):

        self.get_logger().info(
            "----------------------------------------------"
        )

        self.get_logger().info(
            "ADAPTIVE CONTROLLER V2"
        )

        self.get_logger().info(
            f"Network state : "
            f"{self.network_state}"
        )

        self.get_logger().info(
            f"QoS score     : "
            f"{self.qos_score:.2f}/100"
        )

        self.get_logger().info(
            f"  FPS score   : "
            f"{self.fps_score:.2f}"
        )

        self.get_logger().info(
            f"  Loss score  : "
            f"{self.loss_score:.2f}"
        )

        self.get_logger().info(
            f"  Jitter score: "
            f"{self.jitter_score:.2f}"
        )

        self.get_logger().info(
            f"  Frame score : "
            f"{self.frame_age_score:.2f}"
        )

        self.get_logger().info(
            f"  CPU score   : "
            f"{self.cpu_score:.2f}"
        )

        self.get_logger().info(
            "----------------------------------------------"
        )

        self.get_logger().info(
            f"Network:"
            f" FPS={self.actual_fps:.2f}, "
            f"Loss={self.rtp_loss_percent:.2f}%, "
            f"Jitter={self.rtp_jitter_ms:.2f} ms"
        )

        self.get_logger().info(
            f"Video:"
            f" {self.width}x{self.height}, "
            f"target={self.target_fps} FPS, "
            f"Mbps={self.video_mbps:.2f}"
        )

        if self.pi_metrics_received:

            self.get_logger().info(
                f"Pi:"
                f" decode={self.pi_decode_fps:.2f} FPS, "
                f"frame_age={self.pi_frame_age_ms:.2f} ms, "
                f"CPU={self.pi_cpu_percent:.2f}%, "
                f"active={self.pi_video_active:.0f}"
            )

        else:

            self.get_logger().info(
                "Pi: waiting for /qos/pi_metrics"
            )

        self.get_logger().info(
            f"Adaptive FSM:"
            f" {self.adaptive_profile}"
        )

        self.get_logger().info(
            f"Current:"
            f" {self.current_profile} "
            f"{self.current_mode} "
            f"{self.current_width}x"
            f"{self.current_height} "
            f"@{self.current_fps} FPS"
        )

        self.get_logger().info(
            f"Desired:"
            f" {self.desired_profile} "
            f"{self.desired_mode} "
            f"{self.desired_width}x"
            f"{self.desired_height} "
            f"@{self.desired_fps} FPS"
        )

        self.get_logger().info(
            f"Hysteresis:"
            f" HIGH={self.high_count},"
            f" MEDIUM={self.medium_count},"
            f" LOW={self.low_count}"
        )

        self.get_logger().info(
            "----------------------------------------------"
        )

    # ================================================================
    # Controller loop
    # ================================================================

    def controller_loop(self):

        # ------------------------------------------------------------
        # Wait for Notebook QoS metrics
        # ------------------------------------------------------------

        if not self.metrics_received:

            self.get_logger().warning(
                "Waiting for /qos/metrics ..."
            )

            return

        # ------------------------------------------------------------
        # Calculate QoS score
        # ------------------------------------------------------------

        self.calculate_qos_score()

        # ------------------------------------------------------------
        # Evaluate state
        # ------------------------------------------------------------

        self.network_state = (
            self.evaluate_network()
        )

        # ------------------------------------------------------------
        # Make adaptive decision
        # ------------------------------------------------------------

        self.make_decision()

        # ------------------------------------------------------------
        # Publish state
        # ------------------------------------------------------------

        self.publish_state()

        # ------------------------------------------------------------
        # Publish score
        # ------------------------------------------------------------

        self.publish_score()

        # ------------------------------------------------------------
        # Publish decision only when needed
        # ------------------------------------------------------------

        if self.check_decision_change():

            self.print_decision()

            self.publish_decision()

        # ------------------------------------------------------------
        # Debug line every cycle
        # ------------------------------------------------------------

        self.get_logger().info(
            f"[ADAPTIVE V2] "
            f"state={self.network_state} | "
            f"score={self.qos_score:.2f} | "
            f"current={self.current_profile} | "
            f"decision={self.desired_profile} | "
            f"FPS={self.actual_fps:.2f} | "
            f"loss={self.rtp_loss_percent:.2f}% | "
            f"jitter={self.rtp_jitter_ms:.2f} ms | "
            f"frame_age={self.pi_frame_age_ms:.2f} ms | "
            f"CPU={self.pi_cpu_percent:.2f}%"
        )

    # ================================================================
    # Main
    # ================================================================


def main(args=None):
    rclpy.init(args=args)

    node = AdaptiveControllerV2()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info(
                "Adaptive Controller V2 shutting down"
            )

    finally:
        if node is not None:
            node.destroy_node()
            
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":

    main()