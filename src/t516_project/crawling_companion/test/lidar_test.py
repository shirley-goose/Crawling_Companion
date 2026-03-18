import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data  # 💡 新增 1：导入传感器专用的 QoS 配置
import math

class LidarTester(Node):
    def __init__(self):
        super().__init__('lidar_tester')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data)  # 💡 新增 2：把原来的 10 替换成这个配置
            
        self.get_logger().info('雷达测试节点已启动！请在机器人周围走动...')
        self.log_counter = 0

    def scan_callback(self, msg):
        # 雷达通常是 5Hz (每秒扫5圈)，为了不刷屏，我们每扫 5 次打印一次结果 (约1秒1次)
        self.log_counter += 1
        if self.log_counter % 5 != 0: 
            return

        ranges = msg.ranges
        
        # 核心逻辑 1：清理无效数据 (过滤掉太近的自身遮挡噪点、无穷大和 NaN)
        valid_ranges = [(i, r) for i, r in enumerate(ranges) if 0.16 < r < 3.5 and not math.isinf(r) and not math.isnan(r)]
        
        if not valid_ranges:
            self.get_logger().info('雷达没有扫到任何有效物体！')
            return

        # 核心逻辑 2：寻找 360 度内最近的物体
        closest_angle, closest_dist = min(valid_ranges, key=lambda x: x[1])
        
        # 核心逻辑 3：检测正后方 (取 160度 到 200度 扇区)
        rear_ranges = [r for i, r in valid_ranges if 160 <= i <= 200]
        rear_dist = min(rear_ranges) if rear_ranges else 999.0

        # 判断物体大致方位
        if 315 <= closest_angle or closest_angle < 45:
            direction = "正前方 ⬆️"
        elif 45 <= closest_angle < 135:
            direction = "左侧 ⬅️"
        elif 135 <= closest_angle < 225:
            direction = "正后方 ⬇️"
        else:
            direction = "右侧 ➡️"

        # 打印测试结果
        log_msg = f'\n📍 最近物体方位: {closest_angle}° ({direction}), 距离: {closest_dist:.2f} 米'
        if rear_dist < 0.2:
            log_msg += f'\n🚨 警告！背后有障碍物！距离: {rear_dist:.2f} 米 (防撞护盾触发！)'
        else:
            log_msg += f'\n✅ 背后安全，最近后方距离: {rear_dist:.2f} 米'
        
        self.get_logger().info(log_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()