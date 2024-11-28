import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node  # 修改这一行，改为从 launch_ros 导入 Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 你可以通过 launch 参数传递设备路径、分辨率等
        DeclareLaunchArgument(
            'device', default_value='/dev/video0', description='摄像头设备路径'
        ),
        DeclareLaunchArgument(
            'width', default_value='640', description='图像宽度'
        ),
        DeclareLaunchArgument(
            'height', default_value='480', description='图像高度'
        ),

        # 启动 camera_publisher 节点，并传递参数
        Node(
            package='camera_publisher',  # ROS 2 包名
            executable='camera_publisher',  # 可执行文件名
            name='camera_publisher',  # 节点名称
            output='screen',  # 输出到屏幕
            parameters=[{
                'device': LaunchConfiguration('device'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height')
            }],
            # remappings=[('/camera/image_raw', '/camera/image_raw')]  # 根据需要设置话题重映射
        )
    ])
