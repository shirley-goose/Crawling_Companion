import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import socket
import time

class WanderTester(Node):
    def __init__(self):
        super().__init__('wander_tester')
        
        # --- 🔧 参数调整区 ---
        self.target_ip = "10.155.234.247"
        self.LINEAR_SPEED = 0.05      
        self.TURN_SPEED = 0.3         
        self.CRITICAL_DISTANCE = 1 # 全方位：只要有物体进入 25cm 立即锁死 (车体直径30cm)
        self.PATH_DISTANCE = 0.8      # 逻辑前方：提前 50cm 发现障碍
        self.WANDER_TIME = 5.0        
        self.TURN_TIME = 2.0          

        # --- 状态变量 ---
        self.start_time = time.time()
        self.is_turning = False
        self.collision_risk = False    # 全方位风险
        self.path_blocked = False     # 前方路径阻塞
        self.min_dist_all = 999.0
        
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 使用专用 QoS 订阅雷达
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("🛡️ 全方位避障模式启动 (物理后方为逻辑前方)")

    def scan_callback(self, msg):
        # 1. 检查全方位 360 度 (Emergency Bubble)
        all_ranges = [r for r in msg.ranges if 0.05 < r < 3.5]
        if all_ranges:
            self.min_dist_all = min(all_ranges)
            self.collision_risk = self.min_dist_all < self.CRITICAL_DISTANCE
        else:
            self.min_dist_all = 999.0
            self.collision_risk = False

        # 2. 检查逻辑前方 (物理 140-220 度)
        front_ranges = [r for r in msg.ranges[140:220] if 0.05 < r < 3.5]
        if front_ranges:
            self.path_blocked = min(front_ranges) < self.PATH_DISTANCE
        else:
            self.path_blocked = False

    def send_light(self, color_cmd):
        try:
            self.udp_sock.sendto(color_cmd.encode(), (self.target_ip, 9999))
        except:
            pass

    def control_loop(self):
        twist = Twist()
        now = time.time()
        elapsed = now - self.start_time
        
        # 灯光逻辑：正常绿灯，阻塞黄灯，极近红灯（或蓝灯）
        if self.collision_risk:
            self.send_light("BLUE") # 距离太近，显示蓝色报警
        elif self.path_blocked:
            self.send_light("YELLOW") # 路径受阻
        else:
            self.send_light("GREEN")

        # 打印状态调试
        if int(now * 10) % 10 == 0:
            self.get_logger().info(f"最近物体: {self.min_dist_all:.2f}m | 风险: {self.collision_risk}")

        # --- 控制逻辑 ---
        # 只要任何地方有危险，或者前进路径被堵，就停下
        if self.collision_risk or self.path_blocked:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            if self.collision_risk:
                self.get_logger().warn("🚨 全方位碰撞预警！停止所有动作")
        
        elif not self.is_turning:
            if elapsed < self.WANDER_TIME:
                twist.linear.x = -self.LINEAR_SPEED # 物理后退（逻辑前进）
                twist.angular.z = 0.0
            else:
                self.is_turning = True
                self.start_time = now
        else:
            if elapsed < self.TURN_TIME:
                twist.linear.x = 0.0
                twist.angular.z = self.TURN_SPEED
            else:
                self.is_turning = False
                self.start_time = now
        
        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = WanderTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cmd_pub.publish(Twist())
        node.send_light("OFF")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()