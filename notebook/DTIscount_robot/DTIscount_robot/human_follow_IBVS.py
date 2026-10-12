import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .pid import PIDController
from std_msgs.msg import UInt8
from std_msgs.msg import Bool
from my_robot_interfaces.msg import TargetInfo
from collections import deque
import statistics
import numpy as np
from filterpy.kalman import KalmanFilter

class HumanFollowController(Node):
    def __init__(self):
        super().__init__("human_follow_controller")
        self.control_mode = 0
        self.camera_reverse = True
        self.reverse_mode = False
        self.cmd_pub = self.create_publisher(
            Twist,
            "/follow_cmd_vel",
            10
        )
        
        self.mode_sub = self.create_subscription(
            UInt8,
            "/control_mode",
            self.mode_callback,
            10
        )
        
        self.target_sub = self.create_subscription(
            TargetInfo,
            "/target_info",
            self.target_callback,
            10
        )
        
        self.reverse_sub = self.create_subscription(
            Bool,
            "/reverse_mode",
            self.reverse_callback,
            10
        )
        
        # =====================================================
        # ROBOT VELOCITY
        # =====================================================
        self.robot_linear_velocity = 0.0
        self.robot_angular_velocity = 0.0
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "/follow_cmd_vel",
            self.robot_velocity_callback,
            10
        )
        
        # =====================================================
        # TARGET WATCHDOG
        # =====================================================
        self.last_target_time = self.get_clock().now()
        self.target_timeout = 0.7
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)
        
        # -------------------------
        # Camera parameter
        # -------------------------
        self.image_width = 640
        self.image_height = 480
        self.image_center_x = 320
        self.image_center_y = 240
        # =====================================================
        # TARGET SAFETY VALIDATION
        # =====================================================
        self.min_target_pose_confidence = 0.5
        self.min_target_bbox_width = 40
        self.min_target_bbox_height = 80
        self.max_target_bbox_width_ratio = 0.90
        self.max_target_bbox_height_ratio = 0.95
        self.min_target_torso = 40.0
        self.max_target_torso = 350.0
        # Target Area (pixel)
        self.target_torso_length = 165.0
        self.target_cy = 240
        # -------------------------
        # IBVS Controller
        # -------------------------
        self.ibvs_lambda_x = 1.3
        self.ibvs_lambda_torso = 1.2
        
        self.ibvs_deadband_x = 0.03
        self.ibvs_deadband_torso = 0.02
        
        self.max_linear_speed = 0.8
        self.max_angular_speed = 1.0

        # -------------------------
        # PID
        # -------------------------
        self.angular_pid = PIDController(
            kp=0.018,
            ki=0.0,
            kd=0.0,
            min_output=-1.0,
            max_output=1.0
        )

        self.linear_pid = PIDController(
            kp=0.2,
            ki=0.0,
            kd=0.0,
            min_output=-1.0,
            max_output=1.0
        )
        
        #-------------------------
        # Follow State
        #-------------------------
        self.follow_state = "ROTATE"
        self.rotate_enter_threshold = 120
        self.rotate_exit_threshold = 60
        self.distance_threshold = 20
        
        #-------------------------
        # Recovery State
        #-------------------------
        self.robot_state = "FOLLOW"
        self.search_speed = 0.3
        
        #-------------------------
        # Bounding
        #-------------------------
        # Median Filter
        self.torso_buffer = deque(maxlen=3)
        self.cx_buffer = deque(maxlen=3)
        self.cy_buffer = deque(maxlen=3)
        
        # EMA Filter
        self.filtered_torso = None
        self.filtered_cx = None
        self.filtered_cy = None
        self.alpha = 0.6
        
        # ------------------------
        # Kalman Filter
        # ------------------------
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        # state; x = [cx, cy, torso_length, vx, vy, v_torso]
        self.kf.F = np.array([
            [1,0,0,1,0,0],
            [0,1,0,0,1,0],
            [0,0,1,0,0,1],
            [0,0,0,1,0,0],
            [0,0,0,0,1,0],
            [0,0,0,0,0,1]
        ], dtype=float)
        
        self.kf.H = np.array([
            [1,0,0,0,0,0],
            [0,1,0,0,0,0],
            [0,0,1,0,0,0]
        ], dtype=float)
        
        self.kf.P *= 100
        self.kf.R *= 5
        self.kf.Q *= 0.05
        self.kf_initialized = False
        
        # Prediction Timeout
        self.lost_count = 0
        self.max_lost_frames = 30
        
        # Bounding Box Stability
        self.last_cx = None
        self.last_cy = None
        self.last_torso = None
        self.max_cx_change = 250
        self.max_cy_change = 100
        self.min_torso_change = 50.0
        self.max_torso_change = 300.0
        self.torso_velocity_gain = 250.0
        self.max_adaptive_velocity = 0.8
        self.stop_cy_margin = 15
        self.max_distance_error = 1.0

    ###################################################
    def stop(self):
        self.angular_pid.reset()
        self.linear_pid.reset()
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)
        self.get_logger().warn("[STOP] linear=0.0 angular=0.0")
        
    ###################################################
    # Search Target
    ###################################################
    def search_target(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = self.search_speed
        self.get_logger().warn(f"[SEARCH] angular={self.search_speed}")
        self.cmd_pub.publish(msg)
      
    ###################################################
    def reverse_callback(self, msg):
        self.reverse_mode = msg.data
        if self.reverse_mode:
            self.camera_reverse = False
        else:
            self.camera_reverse = True
    
    ###################################################
    # Robot velocity callback
    ###################################################
    def robot_velocity_callback(self, msg):
        self.robot_linear_velocity = abs(
            float(msg.linear.x)
        )
        self.robot_angular_velocity = abs(
            float(msg.angular.z)
        )
    
    ###################################################
    # Adaptive validation threshold
    ###################################################
    def get_adaptive_torso_threshold(self):
        # =================================================
        # 1. Current robot velocity
        # =================================================
        velocity = abs(
            self.robot_linear_velocity
        )

        # =================================================
        # 2. Limit velocity
        # =================================================
        velocity = min(
            velocity,
            self.max_adaptive_velocity
        )   

        # =================================================
        # 3. Calculate adaptive threshold
        # =================================================
        threshold = (
            self.min_torso_change
            +
            self.torso_velocity_gain * velocity
        )

        # =================================================
        # 4. Safety limit
        # =================================================
        threshold = min(
            threshold,
            self.max_torso_change
        )

        # =================================================
        # 5. Debug
        # =================================================
        self.get_logger().info(
            f"[ADAPTIVE VALIDATION] "
            f"velocity={velocity:.3f} "
            f"torso_threshold={threshold:.1f}"
        )

        return threshold
            
    ###################################################    
    def median_filter_torso(self, torso_length):
        self.torso_buffer.append(torso_length)
        return statistics.median(self.torso_buffer)
    
    ###################################################    
    def median_filter_cx(self, cx):
        self.cx_buffer.append(cx)
        return statistics.median(self.cx_buffer)

    ###################################################    
    def median_filter_cy(self, cy):
        self.cy_buffer.append(cy)
        return statistics.median(self.cy_buffer)
    
    ##################################################    
    def ema_filter_torso(self, torso_length):
        if self.filtered_torso is None:
            self.filtered_torso = torso_length
        else:
            self.filtered_torso = (self.alpha * torso_length + (1.0 - self.alpha) * self.filtered_torso)
        return self.filtered_torso

    ###################################################
    def ema_filter_cx(self, cx):
        if self.filtered_cx is None:
            self.filtered_cx = cx
        else:
            self.filtered_cx = (self.alpha * cx + (1-self.alpha) * self.filtered_cx)
        return self.filtered_cx

    ###################################################    
    def ema_filter_cy(self, cy):
        if self.filtered_cy is None:
            self.filtered_cy = cy
        else:
            self.filtered_cy = (self.alpha * cy + (1-self.alpha) * self.filtered_cy)
        return self.filtered_cy
    
    ###################################################
    # Kalman update function
    ###################################################
    def kalman_update(self, cx, cy, torso_length):
        self.get_logger().info(f"[KF UPDATE] " f"cx={cx:.1f} " f"cy={cy:.1f} " f"torso={torso_length:.1f}")
        if not self.kf_initialized:
            self.kf.x = np.array([
                cx,
                cy,
                torso_length,
                0.0,
                0.0,
                0.0
            ], dtype=float)
            self.kf_initialized = True
        self.kf.predict()
        self.kf.update(np.array([cx, cy, torso_length]))
        return (float(self.kf.x[0]), float(self.kf.x[1]), float(self.kf.x[2]))            
    
    ###################################################
    # Kalman predict function
    ###################################################
    def kalman_predict(self):
        if not self.kf_initialized:
            return None
        self.kf.predict()
        
        cx = float(self.kf.x[0])
        cy = float(self.kf.x[1])
        torso_length = float(self.kf.x[2])
        
        cx = max(0.0, min(float(self.image_width), cx))
        cy = max(0.0, min(float(self.image_center_y * 2), cy))
        torso_length = max(1.0, min(self.target_torso_length * 3.0, torso_length))
        self.get_logger().info(
            f"[KALMAN PREDICT] "
            f"cx={cx:.1f} "
            f"cy={cy:.1f} "
            f"torso={torso_length:.1f}"
        )
        return cx, cy, torso_length
    
    ###################################################
    # Adaptive gain
    ###################################################    
    def adaptive_gain(self, error_distance, error_x):
        # ------------------------------
        # Linear Gain Scheduling
        # ------------------------------
        distance_error = abs(error_distance)
        kp_linear_min = 0.12
        kp_linear_max = 0.5
        ratio = min(distance_error / self.max_distance_error, 1.0)
        kp_linear = kp_linear_min + (kp_linear_max - kp_linear_min) * ratio
        
        # ------------------------------
        # Angular Gain Scheduling
        # ------------------------------
        angle_error = abs(error_x)
        kp_angular_min = 0.012
        kp_angular_max = 0.045
        max_angle = self.image_center_x      
        ratio = min(angle_error / max_angle, 1.0)
        kp_angular = kp_angular_min + (kp_angular_max - kp_angular_min) * ratio
            
        # upddate PID gain
        self.linear_pid.set_gains(
            kp_linear,
            0.0,
            0.0
        )
        self.angular_pid.set_gains(
            kp_angular,
            0.0,
            0.0
        ) 
        
        self.get_logger().info(
            f"[GAIN] "
            f"KP_L={kp_linear:.3f} "
            f"KP_A={kp_angular:.3f}"
        )           
    
    ###################################################
    # Validate measurement jump
    ###################################################
    def validate_measurement(self, cx, cy, torso_length):
        # =====================================================
        # 1. FIRST MEASUREMENT
        # =====================================================
        if (self.last_cx is None or self.last_cy is None or self.last_torso is None):
            self.last_cx = cx
            self.last_cy = cy
            self.last_torso = torso_length
            self.get_logger().info(
                "[VALID] First measurement"
            )
            return True, cx, cy, torso_length

        # =====================================================
        # 2. CALCULATE CHANGE
        # =====================================================
        jump_x = abs(
            cx - self.last_cx
        )

        jump_y = abs(
            cy - self.last_cy
        )

        jump_torso = abs(
            torso_length - self.last_torso
        )

        # =====================================================
        # 3. ADAPTIVE TORSO THRESHOLD
        # =====================================================
        adaptive_torso_change = (
            self.get_adaptive_torso_threshold()
        )

        # =====================================================
        # 4. DEBUG
        # =====================================================
        self.get_logger().info(
            f"[CHECK] "
            f"dx={jump_x:.1f} "
            f"dy={jump_y:.1f} "
            f"dTorso={jump_torso:.0f} "
            f"maxTorso={adaptive_torso_change:.1f}"
        )
        
        # =====================================================
        # 5. POSITION JUMP
        # =====================================================
        position_jump = (
            jump_x > self.max_cx_change
            or
            jump_y > self.max_cy_change
        )

        # =====================================================
        # 6. TORSO JUMP
        # =====================================================
        torso_jump = (
            jump_torso > adaptive_torso_change
        )

        # =====================================================
        # 7. HARD REJECT
        # =====================================================
        if position_jump:
            self.get_logger().warn(
                "[POSITION JUMP] REJECT"
            )

            return (
                False,
                self.last_cx,
                self.last_cy,
                self.last_torso
            )

        # =====================================================
        # 8. TORSO REJECT
        # =====================================================
        if torso_jump:
            self.get_logger().warn(
                "[TORSO JUMP] REJECT"
            )
            return (
                False,
                self.last_cx,
                self.last_cy,
                self.last_torso
            )

        # =====================================================
        # 9. UPDATE HISTORY
        # =====================================================
        self.last_cx = cx
        self.last_cy = cy
        self.last_torso = torso_length

        # =====================================================
        # 10. ACCEPT
        # =====================================================
        return (
            True,
            cx,
            cy,
            torso_length
        )

        # =====================================================
        # 3. CHECK SUDDEN JUMP
        # =====================================================
        # if (
        #     jump_x > self.max_cx_change
        #     or
        #     jump_y > self.max_cy_change
        #     or
        #     jump_torso > self.max_torso_change
        # ):
        #     self.get_logger().warn(
        #         f"[JUMP] "
        #         f"cx:{self.last_cx:.1f}->{cx:.1f} "
        #         f"cy:{self.last_cy:.1f}->{cy:.1f} "
        #         f"torso:{self.last_torso:.1f}->{torso_length:.1f}"
        #     )

            # =================================================
            # REJECT MEASUREMENT
            # =================================================
            # return (
            #     False,
            #     self.last_cx,
            #     self.last_cy,
            #     self.last_torso
            # )

        # =====================================================
        # 4. ACCEPT MEASUREMENT
        # =====================================================
        # self.last_cx = cx
        # self.last_cy = cy
        # self.last_torso = torso_length

        # return True, cx, cy, torso_length
    
    ###################################################
    # IBVS Controller
    ###################################################
    def ibvs_control(self, cx, torso_length):

        # 1. HORIZONTAL ERROR
        error_x = (cx - self.image_center_x) / self.image_center_x

        # 2. TORSO ERROR
        error_torso = (torso_length - self.target_torso_length) / self.target_torso_length

        # 3. CLAMP
        error_x = max(-1.0, min(1.0, error_x))
        error_torso = max(-1.0, min(1.0, error_torso))

        # 4. DEADBAND
        if abs(error_x) < self.ibvs_deadband_x:
            error_x = 0.0

        if abs(error_torso) < self.ibvs_deadband_torso:
            error_torso = 0.0

        # 5. IBVS CONTROL
        angular = (-self.ibvs_lambda_x * error_x)
        linear = (-self.ibvs_lambda_torso * error_torso)

        # 6. SATURATION
        linear = max(
            -self.max_linear_speed,
            min(self.max_linear_speed, linear)
        )

        angular = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, angular)
        )

        self.get_logger().info(
            f"[IBVS] "
            f"cx={cx:.1f} "
            f"torso={torso_length:.1f} "
            f"Ex={error_x:.3f} "
            f"Etorso={error_torso:.3f} "
            f"linear={linear:.3f} "
            f"angular={angular:.3f}"
        )

        return linear, angular
        
    ###################################################
    # Follow
    ###################################################   
    def follow(self, cx, cy, torso_length, use_prediction=False):
        # =========================================================
        # 1. FOLLOW CONTROL MODE CHECK
        # =========================================================
        self.get_logger().info(f"[FOLLOW] " f"Robot={self.robot_state} " f"Follow={self.follow_state} " f"Prediction={use_prediction}")
        if self.control_mode != 1:
            self.stop()
            return 
        
        ##############################################
        # 2. FILTER
        ##############################################      
        if not use_prediction:
            self.get_logger().info("[FILTER] YOLO Measurement")
            # 2.1 Median Filter-----------------
            torso_length = self.median_filter_torso(torso_length)
            cx = self.median_filter_cx(cx)
            cy = self.median_filter_cy(cy)
            # 2.2 EMA Filter--------------------
            torso_length = self.ema_filter_torso(torso_length) 
            cx = self.ema_filter_cx(cx)
            cy = self.ema_filter_cy(cy)
            # 2.3 Filter Debug------------------
            self.get_logger().info(f"[FILTER] " f"cx={cx:.1f} " f"cy={cy:.1f} " f"torso={torso_length:.1f}")
            
            # 2.4 Measurement Validation-------
            valid_measurement, cx, cy, torso_length = self.validate_measurement(cx, cy, torso_length)
            self.get_logger().info(f"[VALID] "f"valid={valid_measurement} " f"cx={cx:.1f} " f"cy={cy:.1f} " f"torso={torso_length:.1f}")
        else:
            # Prediction mode-------
            self.get_logger().info("[FILTER] Skip (Prediction)")
            valid_measurement=False
        
        ##############################################
        # 3. Kalman Filter
        ##############################################
        if use_prediction:
            # 3.1 Kalman Prediction-------
            self.get_logger().warn("[KALMAN] PREDICT")
            pred = self.kalman_predict() 
            # 3.2 Prediction failed-------- 
            if pred is None:
                self.get_logger().warn(
                    "[KALMAN] Prediction failed"
                )
                self.stop()
                return
            # 3.3 Get prediction target----- 
            cx, cy, torso_length = pred
            self.get_logger().warn(
                f"[KALMAN PREDICT] "
                f"cx={cx:.1f} "
                f"cy={cy:.1f} "
                f"torso={torso_length:.1f}"
            )
        else:
            # 3.4 Valid YOLO measurement----- 
            if valid_measurement:
                self.get_logger().info("[KALMAN] UPDATE")
                cx, cy, torso_length = self.kalman_update(cx, cy, torso_length)
                self.get_logger().info(f"[KALMAN UPDATE] " f"cx={cx:.1f} " f"cy={cy:.1f} " f"torso={torso_length:.1f}")
            else:
                # 3.5 Invalid measurement----- 
                self.get_logger().warn("[KALMAN] INVALID MEASUREMENT -> PREDICT")
                pred = self.kalman_predict()
                # 3.6 Prediction failed------ 
                if pred is None:
                    self.get_logger().warn("[KALMAN] Prediction failed")
                    self.stop()
                    return
                # 3.7 Use predicted target------
                cx, cy, torso_length = pred
                self.get_logger().warn(
                    f"[KALMAN FALLBACK] "
                    f"cx={cx:.1f} "
                    f"cy={cy:.1f} "
                    f"torso={torso_length:.1f}"
                )
        
        # =========================================================
        # 4. LIMIT KALMAN OUTPUT
        # =========================================================
        #
        # Prevent abnormal Kalman values from producing
        # excessively large control commands.
        #

        cx = max(
            0.0,
            min(
                float(self.image_width),
                float(cx)
            )
        )

        cy = max(
            0.0,
            min(
                float(self.image_height),
                float(cy)
            )
        )

        torso_length = max(
            1.0,
            float(torso_length)
        )

        # =========================================================
        # 5. IBVS CONTROL
        # =========================================================
        if not use_prediction:

            # -----------------------------------------------------
            # NORMAL MODE
            # -----------------------------------------------------
            #
            # cx
            #  ↓
            # angular velocity
            #
            # torso length
            #  ↓
            # linear velocity
            #
            # ทั้งสองคำนวณพร้อมกัน
            #
            linear, angular = self.ibvs_control(
                cx,
                torso_length
            )

            self.get_logger().info(
                f"[IBVS NORMAL] "
                f"linear={linear:.3f} "
                f"angular={angular:.3f}"
            )

        else:
            # -----------------------------------------------------
            # PREDICTION MODE
            # -----------------------------------------------------
            #
            # YOLO หาย
            #
            # ใช้ predicted cx เพื่อหมุนหา target
            #
            # ไม่ใช้ predicted area เพื่อเดิน
            # -----------------------------------------------------
            error_x = (
                cx - self.image_center_x
            ) / self.image_center_x

            # -----------------------------------------------------
            # Clamp horizontal error
            # -----------------------------------------------------
            error_x = max(
                -1.0,
                min(
                    1.0,
                    error_x
                )
            )

            # -----------------------------------------------------
            # Deadband
            # -----------------------------------------------------

            if abs(error_x) < self.ibvs_deadband_x:
                error_x = 0.0

            # -----------------------------------------------------
            # Prediction velocity
            # -----------------------------------------------------
            linear = 0.0
            angular = (-self.ibvs_lambda_x * error_x)

            # -----------------------------------------------------
            # Angular saturation
            # -----------------------------------------------------
            angular = max(
                -self.max_angular_speed,
                min(
                    self.max_angular_speed,
                    angular
                )
            )

            self.get_logger().warn(
                f"[IBVS PREDICT] "
                f"cx={cx:.1f} "
                f"Ex={error_x:.3f} "
                f"linear={linear:.3f} "
                f"angular={angular:.3f}"
            )

        # =========================================================
        # 6. CAMERA DIRECTION
        # =========================================================
        self.get_logger().info(
            f"[BEFORE REVERSE] "
            f"linear={linear:.3f}"
        )

        if self.camera_reverse:
            self.get_logger().info(
                "[REVERSE LINEAR]"
            )
            linear = -linear

        self.get_logger().info(
            f"[AFTER REVERSE] "
            f"linear={linear:.3f}"
        )

        # =========================================================
        # 7. CREATE TWIST
        # =========================================================
        msg = Twist()
        msg.linear.x = float(
            linear
        )

        msg.angular.z = float(
            angular
        )

        # =========================================================
        # 8. FINAL DEBUG
        # =========================================================
        self.get_logger().warn(
            f"[IBVS CMD] "
            f"cx={cx:.1f} "
            f"torso={torso_length:.1f} "
            f"linear={linear:.3f} "
            f"angular={angular:.3f}"
        )

        # =========================================================
        # 9. PUBLISH
        # =========================================================
        self.cmd_pub.publish(msg)
        
    def mode_callback(self, msg):
        # =====================================================
        # 1. รับ Control Mode
        # =====================================================
        self.control_mode = msg.data
        
        # =====================================================
        # 2. MANUAL MODE
        # =====================================================
        if self.control_mode == 0:
            self.get_logger().warn("[MODE] Manual Mode")
            self.stop()
            # -------------------------------------------------
            # Reset Follow State
            # -------------------------------------------------
            self.follow_state = "ROTATE"
            self.robot_state = "FOLLOW"
            # -------------------------------------------------
            # Reset Lost Counter
            # -------------------------------------------------
            self.lost_count = 0
            # -------------------------------------------------
            # Reset Filters
            # -------------------------------------------------
            self.filtered_cx = None
            self.filtered_cy = None
            self.filtered_torso = None
            self.cx_buffer.clear()
            self.cy_buffer.clear()
            self.torso_buffer.clear()
            # -------------------------------------------------
            # Reset Last Measurement
            # -------------------------------------------------
            self.last_cx = None
            self.last_cy = None
            self.last_torso = None
            self.kf_initialized = False
            self.get_logger().warn("[MODE] Human Follow STOPPED")
            
            return
        
        # =====================================================
        # 3. HUMAN FOLLOW MODE
        # =====================================================
        elif self.control_mode == 1:
            self.get_logger().info("[MODE] Human Following Mode")
            return
        
        # =====================================================
        # 4. UNKNOWN MODE
        # =====================================================
        else:
            self.get_logger().warn(
                f"[MODE] Unknown control mode: "
                f"{self.control_mode}"
            )
            self.stop()
            return
    
    def validate_target_info(self, msg):
        """
        Safety gate ก่อน TargetInfo จะเข้า IBVS
        Torso เป็น measurement หลัก
        BBox ใช้เป็นข้อมูลประกอบ ไม่ใช่ hard reject
        """
        torso_length = float(msg.torso_length)
        pose_confidence = float(msg.pose_confidence)

        # =====================================================
        # 1. Pose confidence
        # =====================================================
        if pose_confidence < self.min_target_pose_confidence:
            self.get_logger().warn(
                f"[TARGET INVALID] "
                f"pose_conf={pose_confidence:.2f}"
            )
            return False

        # =====================================================
        # 2. Torso validity
        # =====================================================
        if (torso_length < self.min_target_torso or torso_length > self.max_target_torso):
            self.get_logger().warn(
                f"[TARGET INVALID] "
                f"torso={torso_length:.1f}"
            )
            return False
        
        # =====================================================
        # 3. CENTER VALIDITY
        # =====================================================
        if (
            msg.cx < 0
            or msg.cx > self.image_width
        ):
            self.get_logger().warn(
                f"[TARGET INVALID] "
                f"cx={msg.cx}"
            )
            return False
        if (
            msg.cy < 0
            or msg.cy > self.image_height
        ):
            self.get_logger().warn(
                f"[TARGET INVALID] "
                f"cy={msg.cy}"
            )
            return False

        # =====================================================
        # 4. VALID
        # =====================================================
        return True
        
    def target_callback(self, msg):
        # =====================================================
        # 1. CURRENT STATE
        # =====================================================
        self.get_logger().info(
            f"[STATE] "
            f"Robot={self.robot_state} "
            f"Lost={self.lost_count}"
        )

        # =====================================================
        # 2. CHECK CONTROL MODE
        # =====================================================
        if self.control_mode != 1:
            self.stop()
            return

        # =====================================================
        # 3. YOLO LOST
        # =====================================================
        if not msg.target_found:
            # =================================================
            # SEARCHING STATE
            # =================================================
            if msg.searching:
                self.robot_state = "SEARCH"
                self.lost_count = 0
                self.get_logger().warn(
                    "[STATE] SEARCHING - WAITING FOR USER")
                self.search_target()
                return
            
            # =================================================
            # TARGET LOST
            # =================================================
            self.lost_count += 1
            self.get_logger().warn(
                f"[YOLO LOST] "
                f"count={self.lost_count}/"
                f"{self.max_lost_frames}"
            )

            # =================================================
            # PREDICTION
            # =================================================
            if self.lost_count <= self.max_lost_frames:
                self.robot_state = "PREDICT"
                self.get_logger().warn(
                    "[STATE] PREDICT"
                )

                self.follow(
                    0.0,
                    0.0,
                    0.0,
                    use_prediction=True
                )
                return
            
            # =================================================
            # SAFETY FALLBACK
            # =================================================
            self.robot_state = "SEARCH"
            self.search_target()
            return
            
            # =================================================
            # 3.2 PREDICTION TIMEOUT
            # =================================================
            # self.robot_state = "SEARCH"
            # self.get_logger().warn(
            #     "[STATE] SEARCH"
            # )
            # self.get_logger().warn(
            #     "[PREDICTION TIMEOUT]"
            # )

            # -------------------------------------------------
            # Search target
            # -------------------------------------------------
            # self.search_target()
            # return

        # =====================================================
        # 4. YOLO DETECTED TARGET
        # =====================================================
        torso_length = float(msg.torso_length)
        pose_confidence = float(msg.pose_confidence)

        self.get_logger().info(
            f"[TARGET DETECTED] "
            f"id={msg.track_id} "
            f"cx={msg.cx:.1f} "
            f"cy={msg.cy:.1f} "
            f"pose_conf={pose_confidence:.2f} "
            f"torso={torso_length:.1f}"
        )
        
        # =====================================================
        # TARGET SAFETY VALIDATION
        # =====================================================
        if not self.validate_target_info(msg):
            self.get_logger().warn(
                "[TARGET REJECTED] "
                "Unsafe measurement"
            )
            self.lost_count += 1

            # ---------------------------------------------
            # อย่าเข้า FOLLOW
            # ---------------------------------------------
            if self.lost_count <= self.max_lost_frames:
                self.robot_state = "PREDICT"
                self.follow(
                    0.0,
                    0.0,
                    0.0,
                    use_prediction=True
                )
            else:
                self.robot_state = "SEARCH"
                self.search_target()

            return

        # =====================================================
        # VALID TARGET -> RESET WATCHDOG
        # =====================================================
        self.last_target_time = self.get_clock().now()

        # =====================================================
        # 5. TARGET RECOVERY
        # =====================================================

        # ถ้าก่อนหน้านี้ robot อยู่ PREDICT หรือ SEARCH
        # แล้ว YOLO กลับมาจับ target ได้
        #
        # reset filter เพื่อไม่ให้ค่าที่ค้างจากช่วง lost
        # กระโดดเข้ามาใน IBVS

        if self.robot_state != "FOLLOW":
            self.get_logger().info(
                "[TARGET RECOVERED]"
            )

            self.get_logger().info(
                "[FILTER RESET]"
            )

            # -------------------------------------------------
            # Reset EMA
            # -------------------------------------------------
            self.filtered_cx = None
            self.filtered_cy = None
            self.filtered_torso = None

            # -------------------------------------------------
            # Reset Median buffers
            # -------------------------------------------------
            self.cx_buffer.clear()
            self.cy_buffer.clear()
            self.torso_buffer.clear()
            self.kf_initialized = False

        # =====================================================
        # 6. FOLLOW STATE
        # =====================================================
        self.robot_state = "FOLLOW"

        # =====================================================
        # 7. RESET LOST COUNTER
        # =====================================================
        self.lost_count = 0

        self.get_logger().info(
            "[STATE] FOLLOW"
        )

        # =====================================================
        # 8. NORMAL IBVS
        # =====================================================

        # YOLO มี target จริง
        #
        # follow() จะทำ:
        #
        #   Median Filter
        #        ↓
        #   EMA Filter
        #        ↓
        #   Validation
        #        ↓
        #   Kalman Update
        #        ↓
        #   IBVS
        #
        #   cx   → angular velocity
        #   area → linear velocity
        #
        # ทั้งสองทำงานพร้อมกัน
        self.follow(
            msg.cx,
            msg.cy,
            torso_length,
            use_prediction=False
        )
        
    def watchdog_callback(self):
        if self.control_mode != 1:
            return
        
        if self.robot_state == "SEARCH":
            return
        
        now = self.get_clock().now()
        elapsed = (now - self.last_target_time).nanoseconds / 1e9
        
        # =====================================================
        # TargetInfo timeout
        # =====================================================
        if elapsed > self.target_timeout:
            self.get_logger().warn(
                "[WATCHDOG] "
                f"No TargetInfo for "
                f"{elapsed:.2f}s -> STOP"
            )
            self.stop()

def main(args=None):

    rclpy.init(args=args)
    node = HumanFollowController()
    node.get_logger().info("Human Follow Controller Started")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("[SHUTDOWN] KeyboardInterrupt -> STOP ROBOT")
        node.stop()
    finally:
        node.get_logger().warn("[SHUTDOWN] Publishing final STOP command")
        for _ in range(5):
            node.stop()
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()