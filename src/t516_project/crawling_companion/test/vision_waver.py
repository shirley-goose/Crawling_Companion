import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from ultralytics import YOLO
import math
import time
import cv2
import spidev
import socket

# --- 💡 纯 Python SPI 驱动 (无视 Docker 限制，无视硬件型号) ---
class SPILedStrip:
    def __init__(self, num_leds):
        self.num_leds = num_leds
        self.spi = spidev.SpiDev()
        try:
            self.spi.open(0, 0) # 连接 /dev/spidev0.0
            # 设置 SPI 频率为 6.4 MHz。此时 1 byte = 1.25us，完美契合 WS2812B 周期
            self.spi.max_speed_hz = 6400000 
            self.available = True
        except Exception as e:
            print(f"⚠️ SPI 启动失败: {e}。请确保已在 config.txt 开启 dtparam=spi=on")
            self.available = False
            
        self.pixels = [(0, 0, 0)] * num_leds
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def set_pixel(self, index, r, g, b):
        if 0 <= index < self.num_leds:
            # 限制最高亮度，防止过曝或供电不足 (乘以 0.3 即 30% 亮度)
            self.pixels[index] = (int(r*0.3), int(g*0.3), int(b*0.3))

    def fill(self, r, g, b):
        for i in range(self.num_leds):
            self.set_pixel(i, r, g, b)

    def show(self):
        if not self.available: return
        tx_data = []
        for r, g, b in self.pixels:
            # WS2812B 芯片要求颜色顺序为 GRB
            for color_val in (g, r, b):
                for i in range(7, -1, -1):
                    # 将 1 个颜色位转换为 1 个 SPI 字节 (模拟占空比)
                    # '1' = 11111100 (0xFC), '0' = 11000000 (0xC0)
                    if (color_val >> i) & 1:
                        tx_data.append(0xFC)
                    else:
                        tx_data.append(0xC0)
        # 发送 50us 以上的低电平复位信号 (40 bytes @ 6.4MHz)
        tx_data.extend([0x00] * 40)
        self.spi.xfer2(tx_data)

class VisionWaver(Node):
    def __init__(self):
        super().__init__('vision_waver')

        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.get_logger().info('📡 UDP 通信端口已就绪，目标：127.0.0.1:9999')
        
        # --- 🎯 机械臂挥舞参数 ---
        self.BASE_POSITION = 0.0
        self.ANGLE_DOWN_13 = self.BASE_POSITION + math.radians(13)
        self.ANGLE_UP_17   = self.BASE_POSITION - math.radians(17)
        self.ANGLE_REST    = self.BASE_POSITION
        self.SWING_TIME = 1.0  
        self.joint_name = 'gix'  
        
        self.wave_unlock_time = 0.0 
        self.TOTAL_WAVE_DURATION = self.SWING_TIME * 3 
        
        # --- 🌈 启动无敌版 SPI 灯带 ---
        self.is_waving = False
        self.NUM_LEDS = 30  # 💡 如果你的灯不止 10 颗，请修改这里！
        self.strip = SPILedStrip(self.NUM_LEDS)
        self.strip.fill(0, 0, 0)
        self.strip.show()
        
        self.rainbow_step = 0
        self.led_timer = self.create_timer(0.05, self.led_animation_callback)

        # --- 👁️ 初始化视觉组件 ---
        self.bridge = CvBridge()
        self.get_logger().info('正在加载 YOLOv8 视觉模型...')
        self.model = YOLO('yolov8n.pt')
        
        self.arm_pub = self.create_publisher(JointTrajectory, '/gix_controller/joint_trajectory', 10)
        self.image_sub = self.create_subscription(
            CompressedImage, 
            '/image_raw/compressed', 
            self.image_callback, 
            qos_profile_sensor_data
        )
        self.get_logger().info('✅ 穿墙版 视觉联动挥手+彩虹灯 已启动！')

    def color_wheel(self, pos):
        if pos < 0 or pos > 255: return (0, 0, 0)
        if pos < 85: return (255 - pos * 3, pos * 3, 0)
        if pos < 170:
            pos -= 85
            return (0, 255 - pos * 3, pos * 3)
        pos -= 170
        return (pos * 3, 0, 255 - pos * 3)

    def led_animation_callback(self):
        if time.time() < self.wave_unlock_time:
            # 播放流动彩虹
            for i in range(self.NUM_LEDS):
                pixel_index = (i * 256 // self.NUM_LEDS) + self.rainbow_step
                r, g, b = self.color_wheel(pixel_index & 255)
                self.strip.set_pixel(i, r, g, b)
            self.strip.show()
            self.rainbow_step = (self.rainbow_step + 10) & 255
            self.is_waving = True
        else:
            if self.is_waving:
                self.strip.fill(0, 0, 0)
                self.strip.show()
                self.is_waving = False

    def create_point(self, position, time_float):
        pt = JointTrajectoryPoint()
        pt.positions = [float(position)]
        pt.time_from_start.sec = int(time_float)
        pt.time_from_start.nanosec = int((time_float - int(time_float)) * 1e9)
        return pt

    def send_wave_command(self):
        traj = JointTrajectory()
        traj.header = Header()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = [self.joint_name]  
        
        t1 = self.SWING_TIME * 1
        t2 = self.SWING_TIME * 2
        t3 = self.SWING_TIME * 3
        
        pt1 = self.create_point(self.ANGLE_DOWN_13, t1)
        pt2 = self.create_point(self.ANGLE_UP_17, t2)
        pt3 = self.create_point(self.ANGLE_REST, t3)
        
        traj.points = [pt1, pt2, pt3]
        self.get_logger().info('👋 看到你了！触发动作与绚丽光效...')
        self.arm_pub.publish(traj)
        try:
            self.udp_sock.sendto(b"FLASH", ("10.155.234.247", 9999))
        except Exception as e:
            self.get_logger().error(f"信号发送失败: {e}")

    def image_callback(self, data):
        if time.time() < self.wave_unlock_time: return

        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(data, 'bgr8')
            if len(frame.shape) == 2 or frame.shape[2] == 1:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            
            results = self.model(frame, verbose=False)
            person_detected = any(int(box.cls) == 0 and float(box.conf) > 0.5 for result in results for box in result.boxes)
            
            if person_detected:
                self.send_wave_command()
                self.wave_unlock_time = time.time() + self.TOTAL_WAVE_DURATION

        except Exception as e:
            self.get_logger().error(f'图像处理出错: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = VisionWaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.strip.fill(0, 0, 0)
        node.strip.show()

        reset_traj = JointTrajectory()
        reset_traj.header = Header()
        reset_traj.header.stamp = node.get_clock().now().to_msg()
        reset_traj.joint_names = [node.joint_name]
        
        pt = JointTrajectoryPoint()
        pt.positions = [float(node.BASE_POSITION)]
        pt.time_from_start.sec = 1
        reset_traj.points = [pt]
        
        node.arm_pub.publish(reset_traj)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()