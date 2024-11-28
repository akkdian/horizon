#include "camera_publisher.hpp"
#include <sensor_msgs/msg/image.hpp>  // 假设使用 sensor_msgs/Image

CameraPublisher::CameraPublisher() : Node("camera_publisher") {
    // 初始化发布器
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera_topic", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&CameraPublisher::publishImage, this)
    );
    // 设置共享内存（确保 shm_manager_ 正常初始化）
    shm_manager_.initialize("/camera_shared_memory", IMAGE_SIZE);
}

void CameraPublisher::publishImage() {
    // 获取图像数据（请替换为实际的图像采集逻辑）
    auto image_data = getNV12ImageData();
    if (image_data.empty()) {
        RCLCPP_ERROR(this->get_logger(), "获取图像数据失败.");
        return;
    }
    
    // 将图像数据写入共享内存
    shm_manager_.write(image_data);

    // 创建 ROS 消息并填充数据
    sensor_msgs::msg::Image msg;
    msg.header.stamp = this->now();
    msg.height = IMAGE_HEIGHT;
    msg.width = IMAGE_WIDTH;
    msg.encoding = "nv12";  // 或者使用您实际的图像格式
    msg.step = IMAGE_WIDTH;
    msg.data = image_data;  // 假设 image_data 是 uint8_t 类型的 vector
    
    // 发布图像消息
    publisher_->publish(msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}
