import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import os

class SimpleYoloNode(Node):
    def __init__(self):
        super().__init__('simple_yolo_node')
        self.subscription = self.create_subscription(Image, '/image', self.listener_callback, 10)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('YOLO 识别并截图节点已启动...')

    def listener_callback(self, data):
        try:
            # 将 ROS 图像转换为 OpenCV 格式
            frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            # 执行推理
            results = self.model(frame, verbose=False)
            
            person_detected = False
            for result in results:
                for box in result.boxes:
                    if int(box.cls) == 0:  # Class 0 是人
                        conf = float(box.conf)
                        if conf > 0.5:     # 设置置信度阈值，避免误报
                            person_detected = True
                            # 在图上画出检测框（可选）
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.putText(frame, f'Person {conf:.2f}', (x1, y1-10), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            if person_detected:
                # 保存图像到当前目录
                save_path = os.path.join(os.getcwd(), 'detected_person.jpg')
                cv2.imwrite(save_path, frame)
                self.get_logger().info(f'【已抓拍！】图像已保存至: {save_path}')

        except Exception as e:
            self.get_logger().error(f'处理图像失败: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleYoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
