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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'publish_image_source',
            default_value='./config/test1.nv12',
            description='image source'),
        DeclareLaunchArgument(
            'publish_image_format',
            default_value='nv12',
            description='image format'),
        DeclareLaunchArgument(
            'publish_message_topic_name',
            default_value='/test_msg',
            description='The topic name of message'),
        DeclareLaunchArgument(
            'publish_output_image_w',
            default_value='0',
            description='output image width'),
        DeclareLaunchArgument(
            'publish_output_image_h',
            default_value='0',
            description='output image height'),
        DeclareLaunchArgument(
            'publish_source_image_w',
            default_value='960',
            description='source image width, nv12 format must set'),
        DeclareLaunchArgument(
            'publish_source_image_h',
            default_value='544',
            description='source image height, nv12 format must set'),
        DeclareLaunchArgument(
            'publish_fps',
            default_value='10',
            description='topic publish fps, 1~30'),
        DeclareLaunchArgument(
            'publish_is_loop',
            default_value='True',
            description='loop publish or not'),
        DeclareLaunchArgument(
            'publish_is_shared_mem',
            default_value='True',
            description='using zero copy or not'),
        DeclareLaunchArgument(
            'publish_is_compressed_img_pub',
            default_value='False',
            description='publish jpeg/jpg/png imgs using CompressedImage'),
        # 启动零拷贝环境配置node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_shm'),
                    'launch/hobot_shm.launch.py'))
        ),
        # 启动图片发布pkg，output_image_w与output_image_h设置为0代表不改变图片的分辨率
        Node(
            package='hobot_image_publisher',
            executable='hobot_image_pub',
            output='screen',
            parameters=[
                {"image_source": LaunchConfiguration('publish_image_source')},
                {"image_format": LaunchConfiguration('publish_image_format')},
                {"msg_pub_topic_name": LaunchConfiguration(
                    'publish_message_topic_name')},
                {"output_image_w": LaunchConfiguration(
                    'publish_output_image_w')},
                {"output_image_h": LaunchConfiguration(
                    'publish_output_image_h')},
                {"source_image_w": LaunchConfiguration(
                    'publish_source_image_w')},
                {"source_image_h": LaunchConfiguration(
                    'publish_source_image_h')},
                {"fps": LaunchConfiguration('publish_fps')},
                {"is_loop": LaunchConfiguration('publish_is_loop')},
                {"is_shared_mem": LaunchConfiguration('publish_is_shared_mem')},
                {"is_compressed_img_pub": LaunchConfiguration('publish_is_compressed_img_pub')}
            ],
            arguments=['--ros-args', '--log-level', 'warn']
        )
    ])
