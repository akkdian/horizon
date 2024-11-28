from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # 启动零拷贝环境配置node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_shm'),
                    'launch/hobot_shm.launch.py'))
        ),
        # 启动视频发布pkg
        Node(
            package='hobot_image_publisher',
            executable='hobot_image_pub',
            output='screen',
            parameters=[
                # image_source为本地视频文件的路径或保存本地视频文件路径的list文件，具体使用方法可参考README
                {"image_source": "./config/test1.h264"},
                # image_format为支持发布的本地视频格式，当前支持的格式有h264/h265/mp4
                {"image_format": "h264"},
                {"msg_pub_topic_name": "/test_h264"},
                {"fps": 30},# 优先使用从本地视频文件中解析出来的帧率，解析失败时会采用此fps
                {"is_loop": True},# 是否循环发布  True/False
                {"is_shared_mem": True} # 是否使用share_mem的方式通信, True/False
            ],
            arguments=['--ros-args', '--log-level', 'error']
        )
    ])
