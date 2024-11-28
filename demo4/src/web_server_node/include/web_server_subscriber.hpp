#ifndef WEB_SERVER_SUBSCRIBER_HPP_
#define WEB_SERVER_SUBSCRIBER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "custom_image_message.hpp"  // 假设 CustomImageMessage 类已定义
#include "shared_memory_manager.hpp"  // 假设 SharedMemoryManager 类已定义
#include "httplib.h"  // 用于 HTTP 服务器的头文件

// 常量定义
#define IMAGE_SIZE  (IMAGE_WIDTH * IMAGE_HEIGHT * 3)  // 假设 NV12 格式，每像素 3 字节

class WebServerSubscriber : public rclcpp::Node {
public:
    // 构造函数
    WebServerSubscriber();

private:
    // 订阅回调函数
    void imageCallback(const CustomImageMessage::SharedPtr msg);

    // 订阅器
    rclcpp::Subscription<CustomImageMessage>::SharedPtr subscription_;

    // 共享内存管理器实例
    SharedMemoryManager shm_manager_;  // 负责读取共享内存中的图像数据

    // Web 服务器实例
    httplib::Server svr_;
};

#endif  // WEB_SERVER_SUBSCRIBER_HPP_
