#ifndef CAMERA_PUBLISHER_HPP_
#define CAMERA_PUBLISHER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"  // 使用sensor_msgs/Image消息类型
#include "shared_memory_manager.hpp"  // 假设您有一个共享内存管理器类

// 常量定义
#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480
#define IMAGE_SIZE (IMAGE_WIDTH * IMAGE_HEIGHT * 3)  // 假设每像素 3 字节（例如 NV12 格式）

class CameraPublisher : public rclcpp::Node {
public:
    // 构造函数
    CameraPublisher();

private:
    // 定时器回调函数
    void publishImage();

    // 订阅发布器和定时器
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 共享内存管理器实例
    SharedMemoryManager shm_manager_;  // 假设您有一个负责共享内存的类

    // 获取图像数据的函数（假设该函数从摄像头获取图像）
    std::vector<uint8_t> getNV12ImageData();

    // 可以添加其他成员变量（如摄像头对象、图像格式等）
};

#endif  // CAMERA_PUBLISHER_HPP_
