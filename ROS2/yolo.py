import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. 定位基础包 yolo_bringup
    yolo_pkg = FindPackageShare('yolo_bringup')
    
    # 2. 定义用户特定参数
    model_path = '/home/ubuntu/yolo11m.pt'
    
    # 3. 话题重映射配置
    # 注意：这里我们使用参数传递的方式，而不是 Node 中的 remappings 列表，
    # 因为 yolo_ros 内部将话题名暴露为参数 (input_image_topic 等)。
    
    rgb_topic = '/ascamera/camera_publisher/rgb0/image'
    depth_topic = '/ascamera/camera_publisher/depth0/image_raw'
    # 假设 camera_info 与深度图在同一命名空间，用户需核实
    info_topic = '/ascamera/camera_publisher/depth0/camera_info' 

    # 4. QoS 配置 (假设相机可能使用 Best Effort，若为 Reliable 可设为 1)
    # 为了稳健性，建议先设为 0 (System Default) 或根据实际 topic info 调整
    qos_reliability = '1' # 1=Reliable, 2=Best Effort. 用户需根据 ros2 topic info 结果修改此处

    return LaunchDescription()
            ),
            launch_arguments={
                'model': model_path,
                'model_type': 'YOLO',
                'input_image_topic': rgb_topic,
                'input_depth_topic': depth_topic,
                'input_depth_info_topic': info_topic,
                'use_3d': 'True',
                'target_frame': 'base_link',
                'threshold': '0.5',          # 置信度阈值
                'iou': '0.7',                # NMS 阈值
                'device': 'cpu',          # 启用 GPU
                'half': 'True',              # 启用 FP16 半精度加速
                'image_reliability': qos_reliability,
                'depth_image_reliability': qos_reliability,
                'depth_info_reliability': qos_reliability,
            }.items()
        )
    ])