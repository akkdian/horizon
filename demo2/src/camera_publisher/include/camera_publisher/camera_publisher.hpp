#ifndef CAMERA_PUBLISHER_HPP_
#define CAMERA_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <vector>

struct buffer {
    void* start;
    size_t length;
};

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher();
    ~CameraPublisher();

private:
    void configure_device();
    void capture_and_publish();

    int fd_;  // 摄像头设备的文件描述符
    std::string device_;  // 摄像头设备路径
    int buffer_count_;  // 缓冲区数量
    unsigned int width_;  // 图像宽度
    unsigned int height_;  // 图像高度
    std::vector<struct buffer> buffers_;  // 缓冲区

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;  // ROS 2 图像发布者
    rclcpp::TimerBase::SharedPtr timer_;  // 定时器，定时采集图像
};

#endif  // CAMERA_PUBLISHER_HPP_
