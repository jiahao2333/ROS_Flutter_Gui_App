import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch

class YoloNode(Node):
    def __init__(self):
        super().__init__('ultralytics_node')

        # 检查有没有 GPU，没有就用 CPU
        self.device = '0' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"Using Device: {self.device}")

        # 1. 必须换成 Nano 模型！(yolo11n.pt)
        self.get_logger().info("Loading YOLOv11 Nano model...")
        self.detection_model = YOLO("yolo11n_ncnn_model")
        
        self.bridge = CvBridge()
        self.det_image_pub = self.create_publisher(Image, "/ultralytics/detection/image", 10)

        self.subscription = self.create_subscription(
            Image,
            "/ascamera/camera_publisher/rgb0/image",
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 2. 优化推理参数:
            # - imgsz=640: 限制输入大小，防止高清图卡死
            # - conf=0.4: 过滤掉低置信度的，减少绘图计算量
            # - device=self.device: 强制使用 GPU
            det_result = self.detection_model(
                cv_image, 
                verbose=False, 
                imgsz=640, 
                conf=0.4, 
                device=self.device
            )

            # 3. 绘图 (这一步其实也耗时，如果只要坐标，不要画图，可以把下面两行注释掉)
            det_annotated = det_result[0].plot()
            
            ros_msg = self.bridge.cv2_to_imgmsg(det_annotated, encoding="bgr8")
            self.det_image_pub.publish(ros_msg)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()