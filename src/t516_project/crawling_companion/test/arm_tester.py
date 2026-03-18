import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import math

class ArmTester(Node):
    def __init__(self):
        super().__init__('arm_tester')
        
        # --- 🎯 绝对中心基准 ---
        self.BASE_POSITION = 0.0
        
        # 💡 根据之前的物理规律：加号往下，减号往上
        self.ANGLE_DOWN_5 = self.BASE_POSITION + math.radians(13)   # 往下 5 度 (蓄力)
        self.ANGLE_UP_20 = self.BASE_POSITION - math.radians(17)   # 往上 20 度 (挥舞)
        self.ANGLE_REST = self.BASE_POSITION                       # 回到 0 度 (收势)
        
        # 每段动作的时间 (可以根据你想让它多活泼来调整，1.0~1.5比较合适)
        self.SWING_TIME = 1.0  
        
        self.joint_name = 'gix'  
        self.arm_topic = '/gix_controller/joint_trajectory'
        self.arm_pub = self.create_publisher(JointTrajectory, self.arm_topic, 10)
        
        # 整个动作包含 3 个节点，加 1 秒停顿后循环
        total_wave_time = self.SWING_TIME * 3
        self.timer = self.create_timer(total_wave_time + 0/5, self.send_wave_command)
        
        self.get_logger().info('🦾 蓄力挥手测试已启动！(下压 5° -> 上扬 20° -> 回归 0°)')
        
        self.send_wave_command()

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
        
        # 时间轴安排
        t1 = self.SWING_TIME * 1
        t2 = self.SWING_TIME * 2
        t3 = self.SWING_TIME * 3
        
        # 轨迹顺序：下压 5度 -> 上抬 20度 -> 归零
        pt1 = self.create_point(self.ANGLE_DOWN_5, t1)
        pt2 = self.create_point(self.ANGLE_UP_20, t2)
        pt3 = self.create_point(self.ANGLE_REST, t3)
        
        traj.points = [pt1, pt2, pt3]
        
        self.get_logger().info('👋 正在执行蓄力挥手动作...')
        self.arm_pub.publish(traj)

def main(args=None):
    rclpy.init(args=args)
    node = ArmTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出时安全回到 0.0 (休息位置)
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