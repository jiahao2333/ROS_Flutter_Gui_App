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
    rgb_topic = '/ascamera/camera_publisher/rgb0/image'
    depth_topic = '/ascamera/camera_publisher/depth0/image_raw'
    info_topic = '/ascamera/camera_publisher/depth0/camera_info' 

    # 4. QoS 配置
    qos_reliability = '1' # 1=Reliable, 2=Best Effort

    return LaunchDescription([
        # 修复点：这里需要调用 IncludeLaunchDescription
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                # 假设 yolo_bringup 包内的 launch 文件名为 yolo.launch.py
                # 如果文件名不同（例如 yolo_bringup.launch.py），请在此修改
                PathJoinSubstitution([yolo_pkg, 'launch', 'yolo.launch.py'])
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