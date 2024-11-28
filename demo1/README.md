# Camera Streamer

## 概述

该 ROS2 包通过 V4L2 接口从摄像头设备直接采集 NV12 格式的高分辨率图像数据，使用共享内存和自定义 ROS2 消息实现高效的数据传输，并在 8000 端口上启动一个简单的 HTTP 服务器，以 MJPEG 流的形式在 Web 界面上实时展示图像。

# 运行步骤

##1.运行摄像头发布节点

ros2 run camera_streamer camera_publisher

##2.在新终端中，源化工作空间并运行 Web 显示节点：

source install/setup.bash
ros2 run camera_streamer web_display_node

##3.在浏览器中打开 http://localhost:8000/，即可实时查看
