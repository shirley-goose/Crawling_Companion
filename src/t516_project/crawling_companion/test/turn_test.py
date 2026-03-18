import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, CompressedImage
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from ultralytics import YOLO
import socket, time, math

class BabyInteraction(Node):
    def __init__(self):
        super().__init__('baby_interaction')
        
        # --- 🔧 配置区 ---
        self.target_ip = "10.155.234.247" 
        self.LINEAR_SPEED = 0.06
        self.TURN_SPEED = 0.8          # 互动时的转圈速度
        self.DIST_K = 320.0            
        self.last_seen_time = 0.0       # 👈 新增：最后一次看到人的时间
        self.MEM_DURATION = 2.5         # 👈 新增：记忆持续时间（秒）

        # 距离阈值
        self.RUN_AWAY_DIST = 1.5       # 触发后退的距离
        self.INTERACT_DIST = 2.5       # 停止并开始转圈的距离
        self.APPROACH_MAX = 10.0       

        # 状态变量
        self.state = "IDLE"            # IDLE, APPROACH, INTERACT, RUN_AWAY
        self.detected_dist = 0.0
        self.is_person_present = False
        self.critical_collision = False
        self.last_wave_time = 0.0
        
        # 后退目标计时（用于模拟“后退3m”，大约需要后退 5-8 秒）
        self.run_away_start_time = 0.0
        
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/gix_controller/joint_trajectory', 10)
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.img_sub = self.create_subscription(CompressedImage, '/image_raw/compressed', self.img_cb, qos_profile_sensor_data)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("👶 综合互动逻辑已就绪：接近 -> 2.5m转圈 -> 1.5m逃跑")

    def scan_cb(self, msg):
        # 全方位避障：0.25m 内有任何东西立即锁死
        all_ranges = [r for r in msg.ranges if 0.05 < r < 3.5]
        self.critical_collision = any(r < 0.25 for r in all_ranges)

    def img_cb(self, data):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(data, 'bgr8')
            results = self.model(frame, verbose=False)
            found_this_frame = False
            
            for r in results:
                for box in r.boxes:
                    if int(box.cls) == 0: 
                        h = box.xywh[0][3]
                        self.detected_dist = self.DIST_K / h
                        found_this_frame = True
                        self.last_seen_time = time.time() # 👈 只要看到人，就疯狂刷新记忆时间
                        break
            
            self.is_person_present = found_this_frame
        except: pass

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
        
        # 🛡️ 极近避障（优先级最高，无视记忆）
        if self.critical_collision:
            self.cmd_pub.publish(Twist())
            return

        # 🧠 记忆逻辑：判断“逻辑上”人是否还在
        # 只要 (当前帧看到) 或者 (距离上次看到不满 MEM_DURATION 秒)
        logic_person_present = self.is_person_present or (now - self.last_seen_time < self.MEM_DURATION)

        # --- 状态切换逻辑 ---
        if logic_person_present:
            # 1. 逃跑优先级最高 (1.5m)
            if self.detected_dist < self.RUN_AWAY_DIST:
                if self.state != "RUN_AWAY":
                    self.state = "RUN_AWAY"
                    self.run_away_start_time = now
            
            # 2. 互动优先级 (2.5m) - 增加记忆保护，防止转圈时丢状态
            elif self.detected_dist < self.INTERACT_DIST:
                if self.state != "RUN_AWAY": 
                    self.state = "INTERACT"
            
            # 3. 接近优先级 (10.0m)
            elif self.detected_dist < self.APPROACH_MAX:
                if self.state not in ["RUN_AWAY", "INTERACT"]:
                    self.state = "APPROACH"
        else:
            # 彻底没看到人超过了记忆时间，且没在逃跑，才进入 IDLE
            if self.state != "RUN_AWAY":
                self.state = "IDLE"

        # --- 执行动作 ---
        if self.state == "RUN_AWAY":
            # 逃跑动作不受视觉影响，必须跑完 5 秒
            if now - self.run_away_start_time < 5.0:
                self.send_light("BLUE")
                twist.linear.x = 0.15 
                self.get_logger().info("😱 逃跑中...", throttle_duration_sec=1.0)
            else:
                self.state = "IDLE"

        elif self.state == "INTERACT":
            self.send_light("RAINBOW")
            self.wave_arm()
            twist.angular.z = self.TURN_SPEED
            # 增加一个倒计时打印，方便你看到记忆在生效
            mem_left = self.MEM_DURATION - (now - self.last_seen_time)
            self.get_logger().info(f"💃 互动中 (记忆剩余: {max(0.0, mem_left):.1f}s)", throttle_duration_sec=0.5)

        elif self.state == "APPROACH":
            self.send_light("YELLOW")
            self.wave_arm()
            twist.linear.x = -self.LINEAR_SPEED
            self.get_logger().info(f"🏃 接近中... 距离: {self.detected_dist:.2f}m", throttle_duration_sec=0.5)

        else: # IDLE
            self.send_light("OFF")
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = BabyInteraction()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.cmd_pub.publish(Twist())
        node.send_light("OFF")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()