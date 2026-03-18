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
        
        # --- 🔧 速度与距离核心配置 ---
        self.target_ip = "10.155.234.247" 
        self.LINEAR_SPEED = 0.05
        self.RUN_AWAY_SPEED = 0.15
        self.TURN_SPEED = 0.8
        self.WANDER_TURN_SPEED = 0.3
        self.DIST_K = 320.0            

        # --- 📏 距离阈值定义 ---
        # 社交距离
        self.RUN_AWAY_DIST = 1.5       
        self.INTERACT_DIST = 2.5       
        self.APPROACH_MAX = 10.0       
        # 漫游避障距离
        self.WANDER_CRITICAL_DIST = 0.5 # 漫游全方位预警
        self.WANDER_PATH_DIST = 0.8     # 漫游前方预警
        self.HARD_STOP_DIST = 0.25      # 绝对刹车底线

        # --- 🧠 状态与记忆变量 ---
        self.state = "WANDER"           # 默认状态改为漫游 (WANDER)
        self.detected_dist = 0.0
        self.last_seen_time = 0.0      
        self.MEM_DURATION = 5.0       
        self.is_person_present = False
        
        # 动作计时器
        self.last_wave_time = 0.0
        self.run_away_start_time = 0.0
        self.wander_start_time = time.time()
        self.is_wander_turning = False
        self.WANDER_TIME = 5.0
        self.WANDER_TURN_TIME = 2.0

        # 雷达状态
        self.critical_collision = False
        self.wander_collision_risk = False
        self.wander_path_blocked = False
        self.min_dist_all = 999.0
        
        # --- 📸 调试配置 ---
        self.save_dir = os.path.join(base_dir, "debug_photos")
        self.last_photo_time = 0.0
        self.photo_interval = 5.0  # 👈 更改为每 5 秒保存一次
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        # 初始化工具与通讯
        self.bridge = CvBridge()
        self.get_logger().info("正在初始化 YOLOv8 模型...")
        model_path = os.path.join(base_dir, "yolov8n.pt")
        self.model = YOLO(model_path)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/gix_controller/joint_trajectory', 10)
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.img_sub = self.create_subscription(CompressedImage, '/image_raw/compressed', self.img_cb, qos_profile_sensor_data)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("✅ 终极全功能版：漫游 + 避障 + 视觉社交 已启动！")

    def scan_cb(self, msg):
        # 1. 360度全方位雷达数据
        all_ranges = [r for r in msg.ranges if 0.05 < r < 3.5]
        if all_ranges:
            self.min_dist_all = min(all_ranges)
            self.critical_collision = self.min_dist_all < self.HARD_STOP_DIST
            self.wander_collision_risk = self.min_dist_all < self.WANDER_CRITICAL_DIST
        else:
            self.min_dist_all = 999.0
            self.critical_collision = False
            self.wander_collision_risk = False

        # 2. 逻辑前方雷达数据 (物理后方 140-220度)
        front_ranges = [r for r in msg.ranges[140:220] if 0.05 < r < 3.5]
        self.wander_path_blocked = bool(front_ranges and min(front_ranges) < self.WANDER_PATH_DIST)
    
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
                        
                        # --- 🎯 严格的宝宝过滤逻辑 ---
                        if aspect_ratio >= 0.92: 
                            self.detected_dist = self.DIST_K / h
                            self.last_seen_time = current_time
                            found = True
                            
                            # 记录检测细节
                            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
                            label = f"BABY: {self.detected_dist:.2f}m (AR:{aspect_ratio:.2f})"
                            cv2.putText(annotated_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        else:
                            # 丢弃非目标对象
                            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 0, 255), 1)
                            label_drop = f"Drop (AR:{aspect_ratio:.2f})"
                            cv2.putText(annotated_frame, label_drop, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

            self.is_person_present = found

            # --- 📸 照片保存逻辑 (每 5 秒) ---
            if found and (current_time - self.last_photo_time > self.photo_interval):
                timestamp = time.strftime("%H%M%S")
                filename = f"{self.save_dir}/detect_{timestamp}.jpg"
                cv2.imwrite(filename, annotated_frame)
                self.get_logger().info(f"📸 [视觉快门] 成功锁定目标！照片已保存至: {filename}")
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
            p2.positions, p2.time_from_start.nanosec = [math.radians(-25)], 800000000
            traj.points = [p1, p2]
            self.arm_pub.publish(traj)
            self.last_wave_time = now

    def control_loop(self):
        twist = Twist()
        now = time.time()
        
        # 1. 优先级最高：硬刹车底线
        if self.critical_collision:
            self.cmd_pub.publish(Twist())
            if int(now * 10) % 10 == 0:
                self.get_logger().warn(f"🚨 绝对安全距离触发！障碍物距离: {self.min_dist_all:.2f}m")
            return

        # 2. 核心状态机判断
        logic_present = self.is_person_present or (now - self.last_seen_time < self.MEM_DURATION)

        if logic_present:
            # 看到宝宝：进入社交逻辑
            if self.detected_dist < self.RUN_AWAY_DIST:
                if self.state != "RUN_AWAY":
                    self.state = "RUN_AWAY"
                    self.run_away_start_time = now
            elif self.detected_dist < self.INTERACT_DIST:
                if self.state != "RUN_AWAY": self.state = "INTERACT"
            elif self.detected_dist < self.APPROACH_MAX:
                if self.state not in ["RUN_AWAY", "INTERACT"]: self.state = "APPROACH"
        else:
            # 没看到宝宝：切换回漫游逻辑
            if self.state != "RUN_AWAY":
                if self.state != "WANDER":
                    self.state = "WANDER"
                    self.wander_start_time = now # 重置漫游循环
                    self.is_wander_turning = False

        # --- 📝 日志输出模块 ---
        if int(now * 10) % 10 == 0: # 每秒打印一次状态
            baby_info = f"{self.detected_dist:.2f}m" if logic_present else "未检测到"
            log_msg = f"👉 状态: [{self.state}] | 宝宝距离: {baby_info} | 最近雷达: {self.min_dist_all:.2f}m"
            self.get_logger().info(log_msg)

        # 3. 执行状态动作
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
            
            # 漫游时遇到障碍物：停下并亮警告灯
            if self.wander_collision_risk or self.wander_path_blocked:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.send_light("YELLOW" if self.wander_path_blocked else "BLUE")
                
            # 正常漫游：直走
            elif not self.is_wander_turning:
                if wander_elapsed < self.WANDER_TIME:
                    twist.linear.x = -self.LINEAR_SPEED
                    self.send_light("GREEN")
                else:
                    self.is_wander_turning = True
                    self.wander_start_time = now
                    
            # 正常漫游：转弯
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
