import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge  # 替代 ros_numpy
from ultralytics import YOLO

class YoloNode(Node):
    def __init__(self):
        super().__init__('ultralytics_node')

        # 1. 初始化模型
        # 建议：如果显卡显存较小，可以在这里指定 device，例如 device='cpu' 或 device='0'
        self.get_logger().info("Loading YOLO model...")
        self.detection_model = YOLO("yolo11m.pt")
        
        # 2. 初始化图像转换工具
        self.bridge = CvBridge()

        # 3. 创建发布者 (输出识别后的图)
        # queue_size 在 ROS2 中对应 QoS depth，这里设为 10
        self.det_image_pub = self.create_publisher(Image, "/ultralytics/detection/image", 10)

        # 4. 创建订阅者 (输入摄像头图像)
        # 注意：这里订阅的话题是您提供的 /ascamera/camera_publisher/rgb0/image
        self.subscription = self.create_subscription(
            Image,
            "/ascamera/camera_publisher/rgb0/image",
            self.listener_callback,
            10
        )
        self.get_logger().info("YOLO Node Started. Waiting for images...")

    def listener_callback(self, msg):
        try:
            # Step 1: 将 ROS 图像消息转换为 OpenCV 图像 (Numpy array)
            # desired_encoding='bgr8' 是因为 OpenCV 默认使用 BGR 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Step 2: YOLO 推理
            det_result = self.detection_model(cv_image, verbose=False)

            # Step 3: 绘制结果
            # plot() 返回的是一个 BGR 格式的 numpy 数组
            det_annotated = det_result[0].plot()

            # Step 4: 将 OpenCV 图像转换回 ROS 消息并发布
            ros_msg = self.bridge.cv2_to_imgmsg(det_annotated, encoding="bgr8")
            self.det_image_pub.publish(ros_msg)

        except Exception as e:
            self.get_logger().error(f'Error in callback: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    node = YoloNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 销毁节点，释放资源
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()