# Copyright (c) 2022，Horizon Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # 启动零拷贝环境配置node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_shm'),
                    'launch/hobot_shm.launch.py'))
        ),
        # 启动图片发布pkg
        Node(
            package='hobot_image_publisher',
            executable='hobot_image_pub',
            output='screen',
            parameters=[
                # image_source为本地视频文件的路径或保存本地视频文件路径的list文件，具体使用方法可参考README
                {"image_source": "./config/video.list"},
                # image_format为支持发布的本地视频格式，当前支持的格式有h264/h265/mp4
                {"image_format": "mp4"},
                {"msg_pub_topic_name": "/hbmem_img"},
                {"fps": 30},  # 优先使用从本地视频文件中解析出来的帧率，解析失败时会采用此fps
                {"is_loop": True},  # 是否循环发布  True/False
                {"is_shared_mem": True}  # 是否使用share_mem的方式通信, True/False
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        # 启动nv12图片解码&发布pkg
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_codec'),
                    'launch/hobot_codec.launch.py')),
            launch_arguments={
                'codec_in_mode': 'shared_mem',
                'codec_in_format': 'h264',
                'codec_out_mode': 'shared_mem',
                'codec_out_format': 'nv12',
                'codec_sub_topic': '/hbmem_img',
                'codec_pub_topic': '/image_nv12'
            }.items()
        ),
        # 启动jpeg图片编码&发布pkg
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_codec'),
                    'launch/hobot_codec.launch.py')),
            launch_arguments={
                'codec_in_mode': 'shared_mem',
                'codec_in_format': 'nv12',
                'codec_out_mode': 'ros',
                'codec_out_format': 'jpeg',
                'codec_sub_topic': '/image_nv12',
                'codec_pub_topic': '/image_jpeg'
            }.items()
        ),
        # 启动web展示pkg
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('websocket'),
                    'launch/websocket.launch.py')),
            launch_arguments={
                'websocket_image_topic': '/image_jpeg',
                'websocket_only_show_image': 'True'
            }.items()
        )
    ])
