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
                {"image_source": "./config/test1.jpg"},
                {"image_format": "jpg"},
                {"msg_pub_topic_name": "/hbmem_img"},
                {"output_image_w": 960},
                {"output_image_h": 544},
                {"fps": 10},
                {"is_loop": True},
                {"is_shared_mem": True}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        # 启动jpeg图片编码&发布pkg
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_codec'),
                    'launch/hobot_codec_encode.launch.py')),
            launch_arguments={
                'codec_in_mode': 'shared_mem',
                'codec_out_mode': 'ros',
                'codec_sub_topic': '/hbmem_img',
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
