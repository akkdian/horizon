import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 启动参数（如果需要）
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Namespace for the node'
        ),
        
        # 创建并启动 image_subscriber 节点
        Node(
            package='image_subscriber',  # 包名
            executable='image_subscriber',  # 节点可执行文件名
            name='image_subscriber',  # 节点名称
            namespace=LaunchConfiguration('namespace'),  # 从启动参数中获取命名空间
            output='screen',  # 输出到屏幕
            parameters=[],  # 参数，如果需要可以传递
            remappings=[]  # 重映射话题，如果需要可以配置
        ),
        
        # 输出日志信息
        LogInfo(
            condition=None,
            msg="image_subscriber node is up and running."
        ),
    ])
