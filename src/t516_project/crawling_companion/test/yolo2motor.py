import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, CompressedImage
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
from ultralytics import YOLO
from rclpy.qos import qos_profile_sensor_data
import math
import time

class CrawlingCompanionBrain(Node):
    def __init__(self):
        super().__init__('crawling_companion_brain')
        
        # --- 话题配置 ---
        # 1. 修正：使用真实的话题名字
        self.camera_topic = '/image_raw/compressed'
        self.cmd_topic = '/cmd_vel'
        self.arm_topic = '/gix_controller/joint_trajectory'
        self.lidar_topic = '/scan'
        
        # --- FSM 参数配置 ---
        self.CLOSE_THRESHOLD = 0.5 # 视觉太近比例
        self.FAR_THRESHOLD = 0.15   # 视觉太远比例
        self.MAX_LOST_FRAMES = 15   # 丢失目标判定帧数
        
        # --- 状态追踪变量 ---
        self.frames_without_person = 0
        self.current_state = "INIT"
        self.last_state_log = ""
        self.force_turn_until = 0.0 # 用于状态 1.5 的计时器
        
        # 雷达数据缓存
        self.lidar_target_angle = 0.0
        self.lidar_target_dist = 999.0
        self.lidar_front_dist = 999.0
        self.lidar_rear_dist = 999.0
        
        # 手臂状态锁
        self.arm_busy = False 
        self.current_arm_pose = "" # 记录当前手臂姿态，避免疯狂重复发送
        
        # --- 初始化核心组件 ---
        self.bridge = CvBridge()
        self.get_logger().info('正在加载 YOLOv8 模型，准备终极多传感器融合测试...')
        self.model = YOLO('yolov8n.pt') 
        
        # --- 订阅与发布 ---
        # 2. 修正：换上 qos_profile_sensor_data，打通 Wi-Fi 阻塞
        self.image_sub = self.create_subscription(
            CompressedImage, 
            self.camera_topic, 
            self.image_callback, 
            qos_profile_sensor_data
        )
        self.lidar_sub = self.create_subscription(LaserScan, self.lidar_topic, self.scan_callback, qos_profile_sensor_data)
        
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.arm_pub = self.create_publisher(JointTrajectory, self.arm_topic, 10)
        
        self.get_logger().info('✅ 爬行机器人大脑 (YOLO + LiDAR 终极版) 已启动！')

    def log_state(self, state_name, msg):
        """只在状态切换或关键信息变化时打印日志"""
        if self.current_state != state_name or self.last_state_log != msg:
            self.get_logger().info(f"【{state_name}】 {msg}")
            self.current_state = state_name
            self.last_state_log = msg

    # ================= 雷达数据处理 =================
    def scan_callback(self, msg):
        ranges = msg.ranges
        valid_ranges = [(i, r) for i, r in enumerate(ranges) if 0.16 < r < 3.5 and not math.isinf(r) and not math.isnan(r)]
        if not valid_ranges: return
        
        # 1. 提取探索目标 (0.8m 到 3.0m 之间的最近点)
        explore_ranges = [x for x in valid_ranges if 0.8 <= x[1] <= 3.0]
        if explore_ranges:
            self.lidar_target_angle, self.lidar_target_dist = min(explore_ranges, key=lambda x: x[1])
        else:
            self.lidar_target_dist = 999.0

        # 2. 提取正前方距离 (用于判定状态1.5，角度 0-20 或 340-359)
        front_ranges = [r for i, r in valid_ranges if i <= 20 or i >= 340]
        self.lidar_front_dist = min(front_ranges) if front_ranges else 999.0

        # 3. 提取正后方距离 (用于状态4防撞，角度 160-200)
        rear_ranges = [r for i, r in valid_ranges if 160 <= i <= 200]
        self.lidar_rear_dist = min(rear_ranges) if rear_ranges else 999.0

    # ================= 手臂控制逻辑 =================
    def set_arm_pose(self, pose_name):
        """控制手臂姿态：DOWN, UP, WAVE"""
        if self.current_arm_pose == pose_name and pose_name != "WAVE":
            return # 如果已经是这个姿态，不再重复发指令
            
        if pose_name == "WAVE" and self.arm_busy:
            return # 正在挥舞中，忽略新指令
            
        traj = JointTrajectory()
        traj.joint_names = ['gix']
        
        if pose_name == "DOWN":
            pt = JointTrajectoryPoint()
            pt.positions = [0.0]
            pt.time_from_start.sec = 1
            traj.points = [pt]
            self.arm_pub.publish(traj)
            self.current_arm_pose = "DOWN"
            
        elif pose_name == "UP":
            pt = JointTrajectoryPoint()
            pt.positions = [1.0]
            pt.time_from_start.sec = 1
            traj.points = [pt]
            self.arm_pub.publish(traj)
            self.current_arm_pose = "UP"
            
        elif pose_name == "WAVE":
            self.arm_busy = True
            self.current_arm_pose = "WAVE"
            # 挥舞动作：上 -> 中 -> 下 -> 中 (持续约 4 秒)
            pt1 = JointTrajectoryPoint(); pt1.positions = [1.0]; pt1.time_from_start.sec = 1
            pt2 = JointTrajectoryPoint(); pt2.positions = [0.0]; pt2.time_from_start.sec = 2
            pt3 = JointTrajectoryPoint(); pt3.positions = [1.0]; pt3.time_from_start.sec = 3
            pt4 = JointTrajectoryPoint(); pt4.positions = [0.5]; pt4.time_from_start.sec = 4
            traj.points = [pt1, pt2, pt3, pt4]
            self.arm_pub.publish(traj)
            # 设置定时器解锁
            self.timer = self.create_timer(4.5, self.unlock_arm)

    def unlock_arm(self):
        self.arm_busy = False
        self.current_arm_pose = "" # 允许再次触发
        self.timer.cancel()

    # ================= 视觉与核心 FSM 大脑 =================
    def image_callback(self, data):
        try:
            # --- 强制转身拦截机制 (状态 1.5 正在执行中) ---
            if time.time() < self.force_turn_until:
                twist = Twist()
                twist.angular.z = 0.5 # 持续大角度旋转
                self.cmd_pub.publish(twist)
                self.set_arm_pose("DOWN")
                self.log_state("状态 1.5：放弃目标", "我都凑这么近了还没看到脸，转身寻找下一个！")
                return

            # 3. 修正：使用专用的压缩图像解码函数
            frame = self.bridge.compressed_imgmsg_to_cv2(data, 'bgr8')
            frame_width = frame.shape[1]
            results = self.model(frame, verbose=False)
            
            largest_box = None
            max_area = 0
            face_center_x = 0
            
            for result in results:
                for box in result.boxes:
                    if int(box.cls) == 0 and float(box.conf) > 0.5: # 发现人类
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        w = x2 - x1
                        area = w * (y2 - y1)
                        if area > max_area:
                            max_area = area
                            largest_box = w
                            face_center_x = (x1 + x2) / 2
                            
            twist_msg = Twist()
            
            # ================= FSM 状态机执行 =================
            if largest_box is None:
                self.frames_without_person += 1
                if self.frames_without_person > self.MAX_LOST_FRAMES:
                    
                    # 检查是否触发【状态 1.5：识破伪装 / 放弃】
                    if self.lidar_front_dist < 0.8:
                        self.force_turn_until = time.time() + 2.0 # 强制转身 2 秒
                        return # 交给上面的拦截机制执行
                        
                    # 正常执行【状态 1：主动探索】
                    self.set_arm_pose("DOWN")
                    if self.lidar_target_dist < 4.0:
                        # 计算角度差并转向目标
                        error = self.lidar_target_angle if self.lidar_target_angle <= 180 else self.lidar_target_angle - 360
                        twist_msg.linear.x = 0.1
                        twist_msg.angular.z = max(min(error * 0.02, 0.3), -0.3) # P控制微调方向
                        self.log_state("状态 1：主动探索", f"雷达锁定神秘物体 (距离: {self.lidar_target_dist:.1f}m)，缓慢靠近试探...")
                    else:
                        twist_msg.angular.z = 0.3 # 雷达也没看到东西，原地慢慢转圈找
                        self.log_state("状态 1：主动探索", "雷达视野内暂无目标，原地环视寻找...")
                        
            else:
                self.frames_without_person = 0
                face_ratio = largest_box / frame_width
                
                # 计算视觉对中误差 (让机器人正对着婴儿)
                visual_error = (frame_width / 2) - face_center_x
                twist_msg.angular.z = max(min(visual_error * 0.005, 0.2), -0.2)
                
                # 【状态 4：过近退缩 / 防撞】
                if face_ratio > self.CLOSE_THRESHOLD or self.lidar_front_dist < 0.35:
                    self.set_arm_pose("DOWN")
                    if self.lidar_rear_dist < 0.2:
                        twist_msg.linear.x = 0.0
                        self.log_state("状态 4：退缩防撞", f"太近啦！但背后有墙 (后方余量: {self.lidar_rear_dist:.2f}m)，宁死不退！")
                    else:
                        twist_msg.linear.x = -0.15
                        self.log_state("状态 4：过近退缩", "太近啦！安全后退拉开距离...")
                        
                # 【状态 3：甜蜜互动】
                elif self.FAR_THRESHOLD <= face_ratio <= self.CLOSE_THRESHOLD or 0.4 <= self.lidar_front_dist <= 0.6:
                    twist_msg.linear.x = 0.0
                    self.set_arm_pose("WAVE")
                    self.log_state("状态 3：甜蜜互动", "抓到你啦！保持距离，开心挥手！")
                    
                # 【状态 2：惊喜靠近】
                else:
                    twist_msg.linear.x = 0.15
                    self.set_arm_pose("UP")
                    self.log_state("状态 2：惊喜靠近", "确认是小伙伴！高举双手加速冲过去！")
                
            self.cmd_pub.publish(twist_msg)

        except Exception as e:
            self.get_logger().error(f'处理失败: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = CrawlingCompanionBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist()) # 安全停止底盘
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()