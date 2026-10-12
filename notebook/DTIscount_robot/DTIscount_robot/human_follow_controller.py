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
        
        # -------------------------
        # Camera parameter
        # -------------------------
        self.image_width = 640
        self.image_center_x = 320
        self.image_center_y = 240
        # Target Area (pixel)
        self.target_area = 40000
        self.target_cy = 240
        # ----------IBVS-----------
        self.w_area = 1.0
        self.w_cy = 0.0

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
        self.area_buffer = deque(maxlen=3)
        self.cx_buffer = deque(maxlen=3)
        self.cy_buffer = deque(maxlen=3)
        
        # EMA Filter
        self.filtered_area = None
        self.filtered_cx = None
        self.filtered_cy = None
        self.alpha = 0.6
        
        # ------------------------
        # Kalman Filter
        # ------------------------
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        # state; x = [cx, cy, area, vx, vy, varea]
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
        self.max_lost_frames = 10
        
        # Bounding Box Stability
        self.last_cx = None
        self.last_cy = None
        self.last_area = None
        self.max_cx_change = 250
        self.max_cy_change = 100000
        self.max_area_change = 6000
        self.stop_area_margin = 0.98
        self.stop_cy_margin = 15
        self.max_distance_error = 1.0
        self.area_stop_threshold = 0.05

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
    def median_filter(self, area):
        self.area_buffer.append(area)
        return statistics.median(self.area_buffer)
    
    ###################################################    
    def median_filter_cx(self, cx):
        self.cx_buffer.append(cx)
        return statistics.median(self.cx_buffer)

    ###################################################    
    def median_filter_cy(self, cy):
        self.cy_buffer.append(cy)
        return statistics.median(self.cy_buffer)
    
    ##################################################    
    def ema_filter(self, area):
        if self.filtered_area is None:
            self.filtered_area = area
        else:
            self.filtered_area = (self.alpha * area + (1.0 - self.alpha) * self.filtered_area)
        return self.filtered_area

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
    def kalman_update(self, cx, cy, area):
        self.get_logger().info(f"[KF UPDATE] cx={cx:.1f} cy={cy:.1f} area={area:.0f}")
        if not self.kf_initialized:
            self.kf.x = np.array([
                cx,
                cy,
                area,
                0.0,
                0.0,
                0.0
            ], dtype=float)
            self.kf_initialized = True
        self.kf.predict()
        self.kf.update(np.array([cx, cy, area]))
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
        area = float(self.kf.x[2])
        
        cx = max(0.0, min(float(self.image_width), cx))
        cy = max(0.0, min(float(self.image_center_y * 2), cy))
        area = max(500.0, min(self.target_area*3.0, area))
        self.get_logger().info(
            f"[KALMAN PREDICT] "
            f"cx={cx:.1f} "
            f"cy={cy:.1f} "
            f"area={area:.0f}"
        )
        return cx, cy, area
    
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
    # Validate bbox jump
    ###################################################
    def validate_measurement(self, cx, cy, area):
        if self.last_cx is None:
            self.last_cx = cx
            self.last_cy = cy
            self.last_area = area
            return True, cx, cy, area

        jump_x = abs(cx-self.last_cx)
        jump_y = abs(cy-self.last_cy)
        jump_area = abs(area-self.last_area)
        self.get_logger().info(
            f"[CHECK] "
            f"dx={jump_x:.1f} "
            f"dy={jump_y:.1f} "
            f"dArea={jump_area:.0f}"
        )
        
        # if (jump_x > self.max_cx_change or
        #     jump_y > self.max_cy_change or
        #     jump_area > self.max_area_change):
    
            # self.get_logger().warn(
            #     f"[JUMP] "
            #     f"cx:{self.last_cx}->{cx} "
            #     f"cy:{self.last_cy}->{cy} "
            #     f"area:{self.last_area}->{area}"
            # )

            # return False, self.last_cx, self.last_cy, self.last_area

        self.last_cx=cx
        self.last_cy=cy
        self.last_area=area

        return True,cx,cy,area
        
    ###################################################
    # Follow
    ###################################################   
    def follow(self, cx, cy, area, use_prediction=False):
        self.get_logger().info(f"[FOLLOW] Robot={self.robot_state} Follow={self.follow_state}")
        if self.control_mode != 1:
            self.stop()
            return 
        
        ##############################################
        # Bounding Box and cx Filtering
        ##############################################      
        if not use_prediction:
            self.get_logger().info("[FILTER] YOLO Measurement")
            # Median---------------------------
            area = self.median_filter(area)
            cx = self.median_filter_cx(cx)
            cy = self.median_filter_cy(cy)
            # EMA -----------------------------
            area = self.ema_filter(area) 
            cx = self.ema_filter_cx(cx)
            cy = self.ema_filter_cy(cy)
            self.get_logger().info(f"[FILTER] " f"cx={cx:.1f} " f"cy={cy:.1f} " f"area={area:.0f}")
            
            # Bounding Box Validation
            valid_measurement, cx, cy, area = self.validate_measurement(cx, cy, area)
            self.get_logger().info(f"[VALID] area={area:.0f}")
        else:
            self.get_logger().info("[FILTER] Skip (Prediction)")
            valid_measurement=False
        
        ##############################################
        # Kalman Filter
        ##############################################
        if use_prediction:
            self.get_logger().warn("[KALMAN] PREDICT")
            pred = self.kalman_predict()  
            if pred is None:
                self.stop()
                return
            cx, cy, area = pred
        else:
            if valid_measurement:
                self.get_logger().info("[KALMAN] UPDATE")
                cx, cy, area = self.kalman_update(cx, cy, area)
            else:
                self.get_logger().warn("[KALMAN] INVALID MEASUREMENT -> PREDICT")
                pred = self.kalman_predict()
                if pred is None:
                    self.stop()
                    return
                cx, cy, area = pred
        
        ##############################################
        # Horizontal error
        ##############################################
        error_x = self.image_center_x - cx
        # if abs(error_x) < self.rotate_threshold:
        #     error_x = 0
        
        ##############################################
        # Image Based Visual Servoing
        ##############################################
        # # Height error
        # area_error = (self.target_area - area) / self.target_area
        # area_error=max(-1.0,min(1.0,area_error))
        # # Vertical error
        # cy_error = (self.target_cy - cy) / self.image_center_y
        # cy_error=max(-1.0,min(1.0,cy_error))
        # # Fusion
        # error_distance = (self.w_area * area_error + self.w_cy * cy_error)        
        # # Deadband
        # if abs(error_distance) < 0.03:
        #     error_distance = 0.0
        # self.get_logger().info(
        #     f"Area={area:.0f} "
        #     f"AreaErr={area_error:.3f} "
        #     f"CyErr={cy_error:.3f} "
        #     f"Fusion={error_distance:.3f}"
        # )
        area_error = (self.target_area - area) / self.target_area
        area_error = max(-1.0, min(1.0, area_error))
        
        cy_error = (self.target_cy - cy) / self.image_center_y
        cy_error = max(-1.0, min(1.0, cy_error))
        
        error_distance = area_error

        self.get_logger().info(
            f"[DISTANCE] "
            f"area={area:.0f} "
            f"target={self.target_area:.0f} "
            f"error={area_error:.3f}"
        )
        
        # Deadband
        # if abs(error_distance) < 0.03:
        #     error_distance = 0.0

        self.get_logger().info(
            f"Area={area:.0f} "
            f"AreaErr={area_error:.3f} "
            f"CyErr={cy_error:.3f} "
            f"DistanceErr={error_distance:.3f}"
        )
            
        ##############################################
        # Adaptive Gain Scheduling
        ##############################################
        self.adaptive_gain(error_distance, error_x)
        self.get_logger().warn(
            f"[LINEAR INPUT] "
            f"area={area:.0f} "
            f"target_area={self.target_area:.0f} "
            f"area_error={area_error:.3f} "
            f"cy_error={cy_error:.3f} "
            f"error_distance={error_distance:.3f} "
            f"kp={self.linear_pid.kp:.4f}"
        )
        self.get_logger().info(f"[PID GAIN] KP_L={self.linear_pid.kp:.3f} KP_A={self.angular_pid.kp:.3f}")
            
        ##############################################
        # State based control
        ##############################################
        self.get_logger().info(f"[FSM BEFORE] " f"state={self.follow_state} " f"error_x={error_x:.1f} " f"enter={self.rotate_enter_threshold} " f"exit={self.rotate_exit_threshold}")
        angular = 0.0
        linear = 0.0
        
        # ------------------------------
        # Prediction Mode
        # ------------------------------
        if use_prediction:
            self.follow_state = "PREDICT"
            self.get_logger().warn(f"[FSM] PREDICT " f"error_x={error_x:.1f}")
            # หมุนตาม Kalman
            angular = self.angular_pid.update(error_x)
            linear = 0.0
            
        # ------------------------------
        # State 1 : Rotate
        # ------------------------------
        elif self.follow_state == "ROTATE":
            if abs(error_x) <= self.rotate_exit_threshold:
                self.follow_state = "FOLLOW"
                angular = self.angular_pid.update(error_x)
                if abs(area_error) <= self.area_stop_threshold:
                    linear = 0.0
                    #self.get_logger().info(f"[FSM] ROTATE -> FOLLOW")
                else:
                    linear = self.linear_pid.update(area_error)
            else:
                self.follow_state = "ROTATE"
                angular = self.angular_pid.update(error_x)
                linear = 0.0
                self.get_logger().warn(
                    f"[FSM] ROTATE "
                    f"error_x={error_x:.1f} "
                    f"angular={angular:.3f}"
                )
                
        # ------------------------------
        # State 2 : FOLLOW + STEER
        # ------------------------------
        else:
            # -------------------------------
            # Large heading error -> ROTATE
            # -------------------------------
            if abs(error_x) > self.rotate_enter_threshold:
                self.follow_state = "ROTATE"
                angular = self.angular_pid.update(error_x)
                # Stop linear motion while large heading error
                linear = 0.0
                self.get_logger().warn(f"[FSM] FOLLOW -> ROTATE " f"error_x={error_x:.1f}")
            # -------------------------------
            # Normal FOLLOW
            # -------------------------------
            else:
                self.follow_state = "FOLLOW"
                # Always correct heading
                angular = self.angular_pid.update(error_x)
                # Distance control
                if abs(area_error) <= self.area_stop_threshold:
                    linear = 0.0
                    self.get_logger().info(
                        f"[STOP DISTANCE] "
                        f"area={area:.0f} "
                        f"target={self.target_area:.0f} "
                        f"error={area_error:.3f}"
                    )
                else:
                    linear = self.linear_pid.update(area_error)
                    self.get_logger().warn(
                        f"[FOLLOW] "
                        f"area={area:.0f} "
                        f"target={self.target_area:.0f} "
                        f"error={area_error:.3f} "
                        f"linear={linear:.3f}"
                    )
                    # ---------------------------------
                    # Slow down when heading error grows
                    # ---------------------------------
                    heading_ratio = min(abs(error_x)/120.0, 1.0)
                    speed_factor = 1.0 - 0.25 * heading_ratio
                    linear *= speed_factor
                self.get_logger().info(
                    f"[FOLLOW] "
                    f"error_x={error_x:.1f} "
                    f"area_error={area_error:.3f} "
                    f"linear={linear:.3f} "
                    f"angular={angular:.3f}"
                )    
        
        ##############################################
        # Stop when target area is reached
        ##############################################
        # if area >= self.target_area:
        #     self.get_logger().warn(f"[STOP CONDITION] " f"area={area:.0f} " f"target_area={self.target_area:.0f}")
        #     linear = 0.0
        
        ##############################################
        # Camera direction
        ##############################################
        self.get_logger().warn(f"[BEFORE REVERSE] linear={linear:.3f}")    
        if self.camera_reverse:
            self.get_logger().info("[REVERSE LINEAR]")
            linear = -linear
        self.get_logger().warn(f"[AFTER REVERSE] linear={linear:.3f}")
        
        ##############################################
        # Publish Twist
        ##############################################
        self.get_logger().info(f"error_x={error_x:.1f} " f"area_error={area_error:.3f} " f"cy_error={cy_error:.3f} " f"fusion={error_distance:.3f}")
        self.get_logger().info(f"linear={linear:.3f} angular={angular:.3f}")
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.get_logger().warn(f"[CMD]" f" linear={linear:.3f}" f"angular={angular:.3f}")
        self.cmd_pub.publish(msg)
        
    def mode_callback(self, msg):
        self.control_mode = msg.data
        if self.control_mode == 0:
            self.get_logger().info("Manual Mode")
        elif self.control_mode == 1:
            self.get_logger().info("Human Following Mode")
        
    def target_callback(self, msg):
        self.get_logger().info(
            f"[STATE] "
            f"Robot={self.robot_state} "
            f"Lost={self.lost_count}"
        )
        
        if self.control_mode != 1:
            self.stop()
            return
        
        #--------------YOLO LOST-----------------
        if not msg.target_found:
            self.lost_count += 1
            self.get_logger().warn(f"YOLO LOST : {self.lost_count}")
            
            # State : Prediction
            if self.lost_count <= self.max_lost_frames:
                self.robot_state = "PREDICT"
                pred = self.kalman_predict()
                self.get_logger().warn(f"[KALMAN PREDICT] {pred}")
                if pred is None:
                    self.stop()
                    return
                cx, cy, area = pred
                self.follow(cx, cy, area, use_prediction=True)
                return
            
            # State : Search
            self.robot_state = "SEARCH"
            self.get_logger().warn("Prediction Timeout")
            self.search_target()
            return
        
        #--------------YOLO DETECT TARGET---------------
        area = msg.bbox_width * msg.bbox_height
        self.get_logger().info(
            f"[TARGET DETECTED] "
            f"id={msg.track_id} "
            f"cx={msg.cx:.1f} "
            f"cy={msg.cy:.1f} "
            f"width={msg.bbox_width:.1f} "
            f"height={msg.bbox_height:.1f} "
            f"area={area:.0f}"
        )
        
         # Reset filters after recovery
        if self.robot_state != "FOLLOW":
            self.get_logger().info("[FILTER RESET]")
            self.filtered_cx = None
            self.filtered_cy = None
            self.filtered_area = None
            
            self.cx_buffer.clear()
            self.cy_buffer.clear()
            self.area_buffer.clear()
            
        # FOLLOW
        self.robot_state = "FOLLOW"
        self.lost_count = 0
        self.get_logger().info("[STATE] RECOVERY SUCESS")
        self.follow(msg.cx, msg.cy, area, use_prediction=False)

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