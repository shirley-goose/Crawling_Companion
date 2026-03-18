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
        
        # --- 🔧 核心配置 ---
        self.target_ip = "10.155.234.247" 
        self.LINEAR_SPEED = 0.05
        self.RUN_AWAY_SPEED = 0.15
        self.TURN_SPEED = 0.8
        self.DIST_K = 320.0            

        # 距离阈值
        self.RUN_AWAY_DIST = 1.5       
        self.INTERACT_DIST = 2.5       
        self.APPROACH_MAX = 10.0       

        # 状态与记忆
        self.state = "IDLE"            
        self.detected_dist = 0.0
        self.last_seen_time = 0.0      
        self.MEM_DURATION = 2.5        
        self.is_person_present = False
        self.critical_collision = False
        self.last_wave_time = 0.0
        self.run_away_start_time = 0.0
        
        # --- 📸 调试配置 ---
        self.save_dir = "debug_photos"
        self.last_photo_time = 0.0
        self.photo_interval = 2.0  # 每 2 秒保存一次，防止硬盘塞满
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        self.bridge = CvBridge()
        self.get_logger().info("正在初始化 YOLOv8 模型...")
        self.model = YOLO('yolov8n.pt')
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/gix_controller/joint_trajectory', 10)
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.img_sub = self.create_subscription(CompressedImage, '/image_raw/compressed', self.img_cb, qos_profile_sensor_data)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("✅ 调试版婴儿社交逻辑已启动！照片将存入 debug_photos/")

    def scan_cb(self, msg):
        all_ranges = [r for r in msg.ranges if 0.05 < r < 3.5]
        self.critical_collision = any(r < 0.25 for r in all_ranges)
    
    def img_cb(self, data):
        try:
            # 1. 转换图像并进行 YOLO 检测
            frame = self.bridge.compressed_imgmsg_to_cv2(data, 'bgr8')
            
            # 💡 增强 1：加入 conf=0.4，过滤掉低置信度的“胡乱猜测”
            results = self.model(frame, verbose=False, conf=0.4)
            found = False
            
            current_time = time.time()
            annotated_frame = frame.copy() # 创建副本用于画框保存

            for r in results:
                for box in r.boxes:
                    if int(box.cls) == 0: # 识别为人
                        # 获取框的坐标
                        b = box.xyxy[0] 
                        x1, y1, x2, y2 = map(int, b)
                        w = x2 - x1
                        h = y2 - y1
                        
                        # --- 婴儿爬行过滤 ---
                        aspect_ratio = w / h
                        
                        # 🎯 增强 2：采用你的数据结论，将阈值提高到 0.92
                        if aspect_ratio >= 0.92: 
                            self.detected_dist = self.DIST_K / h
                            self.last_seen_time = current_time
                            found = True
                            
                            # 在图上画出识别结果 (成功匹配)
                            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 3) # 改成绿色框表示确认为婴儿
                            label = f"BABY: {self.detected_dist:.2f}m (AR:{aspect_ratio:.2f})"
                            cv2.putText(annotated_frame, label, (x1, y1 - 10), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        else:
                            # 📸 可选调试：画出被丢弃的人形框（红色），方便对比
                            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 0, 255), 1)
                            label_drop = f"Drop (AR:{aspect_ratio:.2f})"
                            cv2.putText(annotated_frame, label_drop, (x1, y1 - 10), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

            self.is_person_present = found

            # 2. 自动保存证据：如果检测到人且满足时间间隔
            if found and (current_time - self.last_photo_time > self.photo_interval):
                timestamp = time.strftime("%H%M%S")
                filename = f"{self.save_dir}/detect_{timestamp}.jpg"
                cv2.imwrite(filename, annotated_frame)
                self.get_logger().info(f"📸 捕获到目标！照片已保存: {filename}")
                self.last_photo_time = current_time

        except Exception as e:
            self.get_logger().error(f"视觉解析失败: {e}")

        except Exception as e:
            self.get_logger().error(f"视觉解析失败: {e}")

    def send_light(self, color):
        try: self.udp_sock.sendto(color.encode(), (self.target_ip, 9999))
        except: pass

    def wave_arm(self):
        now = time.time()
        if now - self.last_wave_time > 1.2:
            traj = JointTrajectory()
            traj.joint_names = ['gix']
            p1 = JointTrajectoryPoint()
            p1.positions = [math.radians(25)]
            p1.time_from_start.nanosec = 400000000
            p2 = JointTrajectoryPoint()
            p2.positions = [math.radians(-25)]
            p2.time_from_start.nanosec = 800000000
            traj.points = [p1, p2]
            self.arm_pub.publish(traj)
            self.last_wave_time = now

    def control_loop(self):
        twist = Twist()
        now = time.time()
        
        if self.critical_collision:
            self.cmd_pub.publish(Twist())
            return

        logic_present = self.is_person_present or (now - self.last_seen_time < self.MEM_DURATION)

        if logic_present:
            if self.detected_dist < self.RUN_AWAY_DIST:
                if self.state != "RUN_AWAY":
                    self.state = "RUN_AWAY"
                    self.run_away_start_time = now
            elif self.detected_dist < self.INTERACT_DIST:
                if self.state != "RUN_AWAY": self.state = "INTERACT"
            elif self.detected_dist < self.APPROACH_MAX:
                if self.state not in ["RUN_AWAY", "INTERACT"]: self.state = "APPROACH"
        else:
            if self.state != "RUN_AWAY": self.state = "IDLE"

        if self.state == "RUN_AWAY":
            if now - self.run_away_start_time < 5.0:
                self.send_light("BLUE")
                twist.linear.x = self.RUN_AWAY_SPEED
            else: self.state = "IDLE"
        elif self.state == "INTERACT":
            self.send_light("RAINBOW")
            self.wave_arm()
            twist.angular.z = self.TURN_SPEED
        elif self.state == "APPROACH":
            self.send_light("YELLOW")
            self.wave_arm()
            twist.linear.x = -self.LINEAR_SPEED
        else:
            self.send_light("OFF")
            twist.linear.x = 0.0
            twist.angular.z = 0.0

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