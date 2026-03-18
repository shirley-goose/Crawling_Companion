import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, CompressedImage
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from ultralytics import YOLO
import socket, time, math, cv2, os

class BabySocialRobot(Node):
    def __init__(self):
        super().__init__('baby_social_robot')
        base_dir = os.path.dirname(os.path.abspath(__file__))
        
        # --- 🔧 Core Speed & Distance Config ---
        self.target_ip = "10.155.234.247" #our turtlebot ip
        self.LINEAR_SPEED = 0.05
        self.RUN_AWAY_SPEED = 0.15
        self.TURN_SPEED = 0.8
        self.WANDER_TURN_SPEED = 0.3
        self.DIST_K = 360.0            

        # --- 📏 Distance Threshold Definitions ---
        # Social distances
        self.RUN_AWAY_DIST = 2.0       
        self.INTERACT_DIST = 3.0       
        self.APPROACH_MAX = 10.0       
        # Wandering obstacle avoidance distances
        self.WANDER_CRITICAL_DIST = 0.3 # Wandering omnidirectional warning
        self.WANDER_PATH_DIST = 0.8     # Wandering front warning
        self.HARD_STOP_DIST = 0.2      # Absolute hard stop baseline

        # --- 🧠 State & Memory Variables ---
        self.state = "WANDER"           # Default state set to WANDER
        self.detected_dist = 0.0
        self.last_seen_time = 0.0      
        self.MEM_DURATION = 5.0       
        self.is_person_present = False
        
        # Action timers
        self.last_wave_time = 0.0
        self.run_away_start_time = 0.0
        self.wander_start_time = time.time()
        self.is_wander_turning = False
        self.WANDER_TIME = 5.0
        self.WANDER_TURN_TIME = 2.0

        # Lidar states
        self.critical_collision = False
        self.wander_collision_risk = False
        self.wander_path_blocked = False
        self.min_dist_all = 999.0
        self.front_dist = 999.0
        self.left_dist = 999.0
        self.right_dist = 999.0
        
        # --- 📸 Debug Configuration ---
        self.save_dir = os.path.join(base_dir, "debug_photos")
        self.last_photo_time = 0.0
        self.photo_interval = 5.0  # 👈 Changed to save every 5 seconds
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        # Initialize tools & communication
        self.bridge = CvBridge()
        self.get_logger().info("Initializing YOLOv8 model...")
        model_path = os.path.join(base_dir, "yolov8n.pt")
        self.model = YOLO(model_path)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/gix_controller/joint_trajectory', 10)
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.img_sub = self.create_subscription(CompressedImage, '/image_raw/compressed', self.img_cb, qos_profile_sensor_data)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("✅ Ultimate Full-Featured Version: Wander + Avoidance + Visual Social Started!")
   
    def scan_cb(self, msg):
        # ==============================
        # 1️⃣ 360° Global Safety Check
        # ==============================
        all_ranges = [r for r in msg.ranges if 0.1 < r < 3.5]

        if all_ranges:
            self.min_dist_all = min(all_ranges)

            # 🚨 Hard stop (Highest priority)
            self.critical_collision = self.min_dist_all < self.HARD_STOP_DIST

            # ⚠️ Wandering risk (Omnidirectional)
            self.wander_collision_risk = self.min_dist_all < self.WANDER_CRITICAL_DIST
        else:
            self.min_dist_all = 999.0
            self.critical_collision = False
            self.wander_collision_risk = False

        # ==============================
        # 2️⃣ Front Check (Decides if it can go straight)
        # ==============================
        front_ranges = [r for r in msg.ranges[140:220] if 0.1 < r < 3.5]

        if front_ranges:
            self.front_dist = min(front_ranges)
            self.wander_path_blocked = self.front_dist < self.WANDER_PATH_DIST
        else:
            self.front_dist = 999.0
            self.wander_path_blocked = False

        # ==============================
        # 3️⃣ Left Check (For detour decision)
        # ==============================
        left_ranges = [r for r in msg.ranges[60:120] if 0.1 < r < 3.5]

        if left_ranges:
            self.left_dist = min(left_ranges)
        else:
            self.left_dist = 999.0

        # ==============================
        # 4️⃣ Right Check (For detour decision)
        # ==============================
        right_ranges = [r for r in msg.ranges[240:300] if 0.1 < r < 3.5]

        if right_ranges:
            self.right_dist = min(right_ranges)
        else:
            self.right_dist = 999.0

    
    def img_cb(self, data):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(data, 'bgr8')
            results = self.model(frame, verbose=False, conf=0.4)
            found = False
            current_time = time.time()
            annotated_frame = frame.copy()

            for r in results:
                for box in r.boxes:
                    if int(box.cls) == 0: 
                        b = box.xyxy[0] 
                        x1, y1, x2, y2 = map(int, b)
                        w = x2 - x1
                        h = y2 - y1
                        
                        aspect_ratio = w / h
                        
                        # --- 🎯 Strict Baby Filtering Logic ---
                        if aspect_ratio >= 0.92: 
                            self.detected_dist = self.DIST_K / h
                            self.last_seen_time = current_time
                            found = True
                            
                            # Record detection details
                            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
                            label = f"BABY: {self.detected_dist:.2f}m (AR:{aspect_ratio:.2f})"
                            cv2.putText(annotated_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        else:
                            # Discard non-target objects
                            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 0, 255), 1)
                            label_drop = f"Drop (AR:{aspect_ratio:.2f})"
                            cv2.putText(annotated_frame, label_drop, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

            self.is_person_present = found

            # --- 📸 Photo Saving Logic (Every 5 seconds) ---
            if found and (current_time - self.last_photo_time > self.photo_interval):
                timestamp = time.strftime("%H%M%S")
                filename = f"{self.save_dir}/detect_{timestamp}.jpg"
                cv2.imwrite(filename, annotated_frame)
                self.get_logger().info(f"📸 [Visual Shutter] Target locked! Photo saved to: {filename}")
                self.last_photo_time = current_time

        except Exception as e:
            pass

    def send_light(self, color):
        try: self.udp_sock.sendto(color.encode(), (self.target_ip, 9999))
        except: pass

    def wave_arm(self):
        now = time.time()
        if now - self.last_wave_time > 1.2:
            traj = JointTrajectory()
            traj.joint_names = ['gix']
            p1, p2 = JointTrajectoryPoint(), JointTrajectoryPoint()
            p1.positions, p1.time_from_start.nanosec = [math.radians(25)], 400000000
            p2.positions, p2.time_from_start.nanosec = [math.radians(-15)], 800000000
            traj.points = [p1, p2]
            self.arm_pub.publish(traj)
            self.last_wave_time = now

    def control_loop(self):
        twist = Twist()
        now = time.time()
        
        # 1. Highest Priority: Hard Stop Baseline
        if self.critical_collision:
            self.cmd_pub.publish(Twist())
            if int(now * 10) % 10 == 0:
                self.get_logger().warn(f"🚨 Absolute Safe Distance Triggered! Obstacle distance: {self.min_dist_all:.2f}m")
            return

        # 2. Core State Machine Judgment
        logic_present = self.is_person_present or (now - self.last_seen_time < self.MEM_DURATION)

        if logic_present:
            # Sees baby: Enter social logic
            if self.detected_dist < self.RUN_AWAY_DIST:
                if self.state != "RUN_AWAY":
                    self.state = "RUN_AWAY"
                    self.run_away_start_time = now
            elif self.detected_dist < self.INTERACT_DIST:
                if self.state != "RUN_AWAY": self.state = "INTERACT"
            elif self.detected_dist < self.APPROACH_MAX:
                if self.state not in ["RUN_AWAY", "INTERACT"]: self.state = "APPROACH"
        else:
            # Does not see baby: Switch back to wandering logic
            if self.state != "RUN_AWAY":
                if self.state != "WANDER":
                    self.state = "WANDER"
                    self.wander_start_time = now # Reset wander loop
                    self.is_wander_turning = False

        # --- 📝 Log Output Module ---
        if int(now * 10) % 10 == 0:

            baby_info = f"{self.detected_dist:.2f}m" if logic_present else "None"

            # Determine current 'sub-behavior'
            sub_state = "IDLE"

            if self.state == "WANDER":
                if self.wander_collision_risk or self.wander_path_blocked:
                    if self.min_dist_all < 0.25:
                        sub_state = "EMERGENCY_AVOID"
                    else:
                        sub_state = "AVOID_TURN"
                elif not self.is_wander_turning:
                    sub_state = "FORWARD"
                else:
                    sub_state = "TURNING"

            elif self.state == "RUN_AWAY":
                sub_state = "ESCAPING"

            elif self.state == "INTERACT":
                sub_state = "SPIN_INTERACT"

            elif self.state == "APPROACH":
                sub_state = "APPROACHING"

            log_msg = (
                f"\n"
                f"🧠 STATE: {self.state} | {sub_state}\n"
                f"👶 Baby: {baby_info} | Seen: {self.is_person_present}\n"
                f"📏 Distances -> Front: {self.front_dist:.2f} | Left: {self.left_dist:.2f} | Right: {self.right_dist:.2f} | Min: {self.min_dist_all:.2f}\n"
                f"🚧 Blocked -> Front: {self.wander_path_blocked} | Risk: {self.wander_collision_risk}\n"
            )

            self.get_logger().info(log_msg)

        # 3. Execute State Actions
        if self.state == "RUN_AWAY":
            if now - self.run_away_start_time < 5.0:
                self.send_light("BLUE")
                twist.linear.x = self.RUN_AWAY_SPEED
            else: self.state = "WANDER"
            
        elif self.state == "INTERACT":
            self.send_light("RAINBOW")
            self.wave_arm()
            twist.angular.z = self.TURN_SPEED
            
        elif self.state == "APPROACH":
            self.send_light("YELLOW")
            self.wave_arm()
            twist.linear.x = -self.LINEAR_SPEED
            
        elif self.state == "WANDER":
            wander_elapsed = now - self.wander_start_time
            
            # Encountered obstacle while wandering: Stop and show warning light
            if self.wander_collision_risk or self.wander_path_blocked:

                if self.min_dist_all < 0.25:
                    twist.linear.x = 0.05
                    twist.angular.z = self.TURN_SPEED if self.left_dist > self.right_dist else -self.TURN_SPEED
                    self.send_light("RED")

                else:
                    twist.linear.x = 0.0
                    twist.angular.z = self.WANDER_TURN_SPEED if self.left_dist > self.right_dist else -self.WANDER_TURN_SPEED
                    self.send_light("PURPLE")

                self.cmd_pub.publish(twist)
                return   # 🚨🚨🚨 CRITICAL!!
                            
            # Normal Wandering: Go straight
            elif not self.is_wander_turning:
                if wander_elapsed < self.WANDER_TIME:
                    twist.linear.x = -self.LINEAR_SPEED
                    self.send_light("GREEN")
                else:
                    self.is_wander_turning = True
                    self.wander_start_time = now
                    
            # Normal Wandering: Turn
            else:
                if wander_elapsed < self.WANDER_TURN_TIME:
                    twist.angular.z = self.WANDER_TURN_SPEED
                    self.send_light("GREEN")
                else:
                    self.is_wander_turning = False
                    self.wander_start_time = now

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = BabySocialRobot()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.cmd_pub.publish(Twist())
        node.send_light("OFF")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()