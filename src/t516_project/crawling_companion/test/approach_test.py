import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, CompressedImage
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from ultralytics import YOLO
import socket, time, math

class ApproachTester(Node):
    def __init__(self):
        super().__init__('approach_tester')
        
        # --- 🔧 Configuration ---
        self.target_ip = "10.155.234.247" 
        self.LINEAR_SPEED = 0.05
        self.STOP_DISTANCE = 2.5   
        self.APPROACH_MAX = 10.0       # Expanded detection range
        self.DIST_K = 320.0            

        self.detected_dist = 0.0
        self.is_person_present = False
        self.obstacle_detected = False
        self.last_wave_time = 0.0
        
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/gix_controller/joint_trajectory', 10)
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.img_sub = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.img_cb, qos_profile_sensor_data)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("🟡 Approach Mode: Waving + Moving + Yellow Light Enabled")

    def scan_cb(self, msg):
        front_ranges = [r for r in msg.ranges[140:220] if 0.1 < r < 3.5]
        self.obstacle_detected = any(r < 0.4 for r in front_ranges)

    def img_cb(self, data):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(data, 'bgr8')
            results = self.model(frame, verbose=False)
            self.is_person_present = False
            for r in results:
                for box in r.boxes:
                    if int(box.cls) == 0: 
                        h = box.xywh[0][3]
                        self.detected_dist = self.DIST_K / h
                        self.is_person_present = True
                        break
        except Exception as e:
            self.get_logger().error(f"Image Error: {e}")

    def send_light(self, color):
        try: 
            self.udp_sock.sendto(color.encode(), (self.target_ip, 9999))
        except: 
            pass

    def wave_arm(self):
        """Sends a waving trajectory command every 1.5 seconds."""
        now = time.time()
        if now - self.last_wave_time > 1.5:
            traj = JointTrajectory()
            traj.joint_names = ['gix']
            p1 = JointTrajectoryPoint()
            p1.positions = [math.radians(20)]
            p1.time_from_start.sec = 0
            p1.time_from_start.nanosec = 400000000 # 0.4s
            p2 = JointTrajectoryPoint()
            p2.positions = [math.radians(-20)]
            p2.time_from_start.sec = 0
            p2.time_from_start.nanosec = 800000000 # 0.8s
            traj.points = [p1, p2]
            self.arm_pub.publish(traj)
            self.last_wave_time = now

    def control_loop(self):
        twist = Twist()
        
        if self.is_person_present:
            self.get_logger().info(f"📏 Distance: {self.detected_dist:.2f}m", throttle_duration_sec=0.5)
            
            if self.STOP_DISTANCE < self.detected_dist < self.APPROACH_MAX:
                self.send_light("YELLOW") # ⚡ Signal for Yellow
                self.wave_arm()           # 👋 Wave while moving
                
                if not self.obstacle_detected:
                    twist.linear.x = -self.LINEAR_SPEED
                    self.get_logger().info("🏃 Approaching...")
                else:
                    self.get_logger().warn("🚧 Blocked!")
            
            elif self.detected_dist <= self.STOP_DISTANCE:
                self.send_light("RAINBOW") # 🌈 Signal for Interaction
                twist.linear.x = 0.0
                self.get_logger().info("📍 Target reached (2.5m). Stopping.")
        else:
            self.send_light("OFF")
            twist.linear.x = 0.0

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ApproachTester()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.cmd_pub.publish(Twist())
        node.send_light("OFF")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()