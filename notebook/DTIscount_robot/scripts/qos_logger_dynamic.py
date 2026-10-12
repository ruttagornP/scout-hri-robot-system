#!/usr/bin/env python3

import argparse
import csv
import os
import signal
import sys
import time
from datetime import datetime, timezone
import psutil

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray


# ============================================================
# DEFAULT
# ============================================================
DEFAULT_MODE = "single"

DEFAULT_OUTPUT = os.path.expanduser(
    "~/qos_logs/video_qos.csv"
)

DEFAULT_INTERVAL = 1.0

# ============================================================
# TIME
# ============================================================
def local_iso_timestamp():

    return datetime.now(
        timezone.utc
    ).astimezone().isoformat(
        timespec="milliseconds"
    )

# ============================================================
# ROS QoS Logger Node
# ============================================================
class QoSLoggerNode(Node):
    def __init__(
        self,
        args
    ):

        super().__init__(
            "qos_logger"
        )

        self.args = args

        # ----------------------------------------------------
        # Current transport mode
        # ----------------------------------------------------
        self.mode = args.mode

        self.segment_id = 0

        self.last_mode = self.mode

        # ----------------------------------------------------
        # Experiment
        # ----------------------------------------------------
        self.experiment_id = (
            args.experiment_id
        )

        # ----------------------------------------------------
        # Timing
        # ----------------------------------------------------
        self.start_time = (
            time.monotonic()
        )

        self.last_log_time = 0.0
        # CPU usage of QoS logger process
        self.process = psutil.Process(os.getpid())
        self.process.cpu_percent(interval=None)
        self.cpu_percent = 0.0

        # ----------------------------------------------------
        # Last QoS metrics
        # ----------------------------------------------------
        self.fps = 0.0
        self.bitrate = 0.0
        self.packet_rate = 0.0

        self.rtp_packets = 0.0
        self.rtp_bytes = 0.0
        self.rtp_lost_packets = 0.0
        self.loss_percent = 0.0
        self.rtp_duplicate_packets = 0.0
        self.rtp_out_of_order_packets = 0.0
        self.jitter_ms = 0.0

        self.width = 0.0
        self.height = 0.0
        self.target_fps = 0.0

        self.ssrc = ""
        self.last_rtp_seq = ""
        self.last_rtp_timestamp = ""
        
        # ----------------------------------------------------
        # Pi Receiver QoS
        # ----------------------------------------------------
        self.pi_decode_fps = 0.0
        self.pi_frame_age_ms = 9999.0
        self.pi_cpu_percent = 0.0
        self.pi_video_active = 0

        # ----------------------------------------------------
        # CSV
        # ----------------------------------------------------
        self.csv_file = None

        self.csv_writer = None

        self.open_csv()

        # ----------------------------------------------------
        # ROS subscriber
        # ----------------------------------------------------
        self.metrics_subscription = (
            self.create_subscription(
                Float64MultiArray,
                "/qos/metrics",
                self.metrics_callback,
                10
            )
        )
        
        # ----------------------------------------------------
        # ROS Pi QoS subscriber
        # ----------------------------------------------------
        self.pi_metrics_subscription = (
            self.create_subscription(
                Float64MultiArray,
                "/qos/pi_metrics",
                self.pi_metrics_callback,
                10
            )
        )

        # ----------------------------------------------------
        # ROS transport mode subscriber
        # ----------------------------------------------------
        self.mode_subscription = (
            self.create_subscription(
                String,
                args.ros_mode_topic,
                self.mode_callback,
                10
            )
        )

        # ----------------------------------------------------
        # Console
        # ----------------------------------------------------
        self.get_logger().info(
            "QoS Logger started"
        )

        self.get_logger().info(
            "Subscribing: /qos/metrics"
        )
        
        self.get_logger().info(
            "Subscribing: /qos/pi_metrics"
        )

        self.get_logger().info(
            "Subscribing: "
            + args.ros_mode_topic
        )

        self.get_logger().info(
            "CSV: "
            + args.output
        )

    # ========================================================
    # Receive Pi QoS from ROS
    # ========================================================
    def pi_metrics_callback(self, msg):

        data = list(msg.data)

        if len(data) < 4:
            self.get_logger().warning(
                "Invalid Pi QoS metrics: "
                f"expected 4 values, got {len(data)}"
            )
            return

        # ----------------------------------------------------
        # Decode Pi QoS metrics
        #
        # [0] decode_fps
        # [1] frame_age_ms
        # [2] cpu_percent
        # [3] video_active
        # ----------------------------------------------------

        self.pi_decode_fps = float(data[0])
        self.pi_frame_age_ms = float(data[1])
        self.pi_cpu_percent = float(data[2])
        self.pi_video_active = int(data[3]) != 0

        self.get_logger().debug(
            f"Pi QoS: "
            f"FPS={self.pi_decode_fps:.2f}, "
            f"Age={self.pi_frame_age_ms:.2f} ms, "
            f"CPU={self.pi_cpu_percent:.2f}%, "
            f"Video={'ON' if self.pi_video_active else 'OFF'}"
        )

    # ========================================================
    # Open CSV
    # ========================================================
    def open_csv(self):

        output_dir = os.path.dirname(
            self.args.output
        )

        if output_dir:
            os.makedirs(
                output_dir,
                exist_ok=True
            )

        file_exists = os.path.exists(
            self.args.output
        )

        self.csv_file = open(
            self.args.output,
            "a",
            newline=""
        )

        self.csv_writer = csv.DictWriter(
            self.csv_file,
            fieldnames=[
                "timestamp",
                "elapsed_s",
                "experiment_id",
                "segment_id",
                "transport_mode",
                "resolution",
                "target_fps",
                "actual_fps",
                "video_mbps",
                "rtp_packet_rate",
                "rtp_packets",
                "rtp_bytes",
                "rtp_lost_packets",
                "rtp_loss_percent",
                "rtp_duplicate_packets",
                "rtp_out_of_order_packets",
                "rtp_jitter_ms",
                "cpu_percent",
                "pi_decode_fps",
                "pi_frame_age_ms",
                "pi_cpu_percent",
                "pi_video_active",
                "ssrc",
                "last_rtp_seq",
                "last_rtp_timestamp",
                "segment_event"
            ]
        )

        if not file_exists:

            self.csv_writer.writeheader()

            self.csv_file.flush()

    # ========================================================
    # Transport mode callback
    # ========================================================
    def mode_callback(
        self,
        msg
    ):

        new_mode = (
            msg.data
            .strip()
            .lower()
        )

        if new_mode not in (
            "single",
            "multicast"
        ):

            self.get_logger().warning(
                "Invalid transport mode: "
                + new_mode
            )

            return

        if new_mode == self.mode:

            return

        old_mode = self.mode

        self.mode = new_mode

        self.segment_id += 1

        self.last_log_time = 0.0

        self.get_logger().info(
            "MODE CHANGE: "
            + old_mode.upper()
            + " -> "
            + new_mode.upper()
        )

    # ========================================================
    # QoS metrics callback
    # ========================================================
    def metrics_callback(
        self,
        msg
    ):
        data = list(msg.data)
        if len(data) < 12:
            self.get_logger().warning(
                "Invalid QoS metrics: "
                f"expected 12 values, got {len(data)}"
            )
            return

        # ----------------------------------------------------
        # Decode GUI QoS metrics
        #
        # [0]  actual_fps
        # [1]  video_mbps
        # [2]  rtp_packet_rate
        # [3]  rtp_packets
        # [4]  rtp_bytes
        # [5]  rtp_lost_packets
        # [6]  rtp_duplicate_packets
        # [7]  rtp_out_of_order_packets
        # [8]  rtp_jitter_ms
        # [9]  width
        # [10] height
        # [11] target_fps
        # ----------------------------------------------------

        self.fps = float(data[0])
        self.bitrate = float(data[1])
        self.packet_rate = float(data[2])

        self.rtp_packets = float(data[3])
        self.rtp_bytes = float(data[4])
        self.rtp_lost_packets = float(data[5])

        self.rtp_duplicate_packets = float(data[6])
        self.rtp_out_of_order_packets = float(data[7])
        self.jitter_ms = float(data[8])

        self.width = float(data[9])
        self.height = float(data[10])
        self.target_fps = float(data[11])

        # ----------------------------------------------------
        # Calculate packet loss percentage
        # ----------------------------------------------------
        total_packets = (
            self.rtp_packets +
            self.rtp_lost_packets
        )
        if total_packets > 0:
            self.loss_percent = (
                self.rtp_lost_packets /
                total_packets
            ) * 100.0

        else:
            self.loss_percent = 0.0

        # ----------------------------------------------------
        # Debug
        # ----------------------------------------------------
        self.get_logger().debug(
            f"QoS: "
            f"FPS={self.fps:.2f}, "
            f"Mbps={self.bitrate:.2f}, "
            f"RTP={self.packet_rate:.2f}, "
            f"packets={self.rtp_packets:.0f}, "
            f"bytes={self.rtp_bytes:.0f}, "
            f"lost={self.rtp_lost_packets:.0f}, "
            f"loss={self.loss_percent:.2f}%, "
            f"dup={self.rtp_duplicate_packets:.0f}, "
            f"ooo={self.rtp_out_of_order_packets:.0f}, "
            f"jitter={self.jitter_ms:.3f} ms, "
            f"resolution={int(self.width)}x{int(self.height)}, "
            f"target={self.target_fps:.1f}"
        )

        # ----------------------------------------------------
        # Write CSV according to logging interval
        # ----------------------------------------------------
        now = time.monotonic()
        if (
            now - self.last_log_time
            >= self.args.interval
        ):
            # Update CPU usage before writing CSV
            self.cpu_percent = self.process.cpu_percent(interval=None)
            self.write_sample()
            self.last_log_time = now

    # ========================================================
    # Write CSV
    # ========================================================
    def write_sample(
        self
    ):

        now = time.monotonic()

        elapsed = (
            now - self.start_time
        )

        resolution = (
            f"{int(self.width)}x"
            f"{int(self.height)}"
        )

        row = {

            "timestamp":
                local_iso_timestamp(),

            "elapsed_s":
                f"{elapsed:.3f}",

            "experiment_id":
                self.experiment_id,

            "segment_id":
                self.segment_id,

            "transport_mode":
                self.mode,

            "resolution":
                resolution,

            "target_fps":
                f"{self.target_fps:.1f}",

            "actual_fps":
                f"{self.fps:.3f}",

            "video_mbps":
                f"{self.bitrate:.6f}",

            "rtp_packet_rate":
                f"{self.packet_rate:.3f}",

            "rtp_packets":
                f"{self.rtp_packets:.0f}",

            "rtp_bytes":
                f"{self.rtp_bytes:.0f}",

            "rtp_lost_packets":
                f"{self.rtp_lost_packets:.0f}",

            "rtp_loss_percent":
                f"{self.loss_percent:.5f}",

            "rtp_duplicate_packets":
                f"{self.rtp_duplicate_packets:.0f}",

            "rtp_out_of_order_packets":
                f"{self.rtp_out_of_order_packets:.0f}",

            "rtp_jitter_ms":
                f"{self.jitter_ms:.4f}",

            "cpu_percent":
                f"{self.cpu_percent:.2f}",
                        
            "pi_decode_fps":
                f"{self.pi_decode_fps:.3f}",

            "pi_frame_age_ms":
                f"{self.pi_frame_age_ms:.3f}",

            "pi_cpu_percent":
                f"{self.pi_cpu_percent:.2f}",

            "pi_video_active":
                self.pi_video_active,

            "ssrc":
                "",

            "last_rtp_seq":
                "",

            "last_rtp_timestamp":
                "",

            "segment_event":
                "SAMPLE"
        }

        if self.csv_writer is not None:

            self.csv_writer.writerow(
                row
            )

            self.csv_file.flush()

        # ----------------------------------------------------
        # Console
        # ----------------------------------------------------
        print(
            "[QOS] "
            f"{self.mode.upper():9s} | "
            f"FPS {self.fps:6.2f} | "
            f"BW {self.bitrate:7.3f} Mbps | "
            f"PKT {self.packet_rate:7.1f}/s | "
            f"LOSS {self.loss_percent:7.3f}% | "
            f"JIT {self.jitter_ms:7.3f} ms | "
            f"CPU {self.cpu_percent:5.1f}% | "
            f"{int(self.width)}x"
            f"{int(self.height)} @ "
            f"{self.target_fps:.1f}"
            f"PI_FPS {self.pi_decode_fps:6.2f} |"
            f"PI_AGE {self.pi_frame_age_ms:7.2f} ms | "
            f"PI_CPU {self.pi_cpu_percent:5.1f}% | "
            f"PI {'ON' if self.pi_video_active else 'OFF'} | "
        )

    # ========================================================
    # Close
    # ========================================================
    def close_csv(
        self
    ):

        if self.csv_file is not None:

            try:

                self.csv_file.flush()

                self.csv_file.close()

            except Exception:

                pass

            self.csv_file = None
            self.csv_writer = None

    # ========================================================
    # Shutdown
    # ========================================================
    def shutdown_logger(
        self
    ):

        self.get_logger().info(
            "QoS Logger shutting down"
        )

        self.close_csv()

# ============================================================
# Arguments
# ============================================================
def parse_arguments():

    parser = argparse.ArgumentParser(
        description=(
            "ROS 2 QoS Metrics Logger"
        )
    )

    parser.add_argument(
        "--mode",
        choices=[
            "single",
            "multicast"
        ],
        default=DEFAULT_MODE
    )

    parser.add_argument(
        "--experiment-id",
        default="EXP001"
    )

    parser.add_argument(
        "--output",
        default=DEFAULT_OUTPUT
    )

    parser.add_argument(
        "--interval",
        type=float,
        default=DEFAULT_INTERVAL
    )

    parser.add_argument(
        "--ros-mode-topic",
        default="/qos/transport_mode"
    )

    return parser.parse_args()

# ============================================================
# Global
# ============================================================
LOGGER_NODE = None

# ============================================================
# Signal
# ============================================================
def signal_handler(
    signum,
    frame
):

    global LOGGER_NODE

    print()

    print(
        "[QOS] Signal "
        + str(signum)
    )

    if LOGGER_NODE is not None:

        LOGGER_NODE.shutdown_logger()

# ============================================================
# Main
# ============================================================
def main():

    global LOGGER_NODE

    args = parse_arguments()

    rclpy.init()

    LOGGER_NODE = (
        QoSLoggerNode(
            args
        )
    )

    signal.signal(
        signal.SIGINT,
        signal_handler
    )

    signal.signal(
        signal.SIGTERM,
        signal_handler
    )

    try:

        rclpy.spin(
            LOGGER_NODE
        )

    except KeyboardInterrupt:

        pass

    finally:

        if LOGGER_NODE is not None:

            LOGGER_NODE.shutdown_logger()

            LOGGER_NODE.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()

if __name__ == "__main__":

    main()