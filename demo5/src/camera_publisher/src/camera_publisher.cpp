#include "camera_publisher/camera_publisher.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <unistd.h>

CameraPublisher::CameraPublisher()
    : Node("camera_publisher"), buffer_count_(4), width_(640), height_(480)  // 设置为640x480
{
    // 打开摄像头设备
    device_ = "/dev/video0";
    fd_ = open(device_.c_str(), O_RDWR);
    if (fd_ == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to open video device");
        return;
    }

    // 配置设备
    configure_device();

    // 创建发布者
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

    // 定时器，每 100 毫秒调用一次图像采集函数
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&CameraPublisher::capture_and_publish, this));
}

CameraPublisher::~CameraPublisher()
{
    // 释放资源
    if (fd_ != -1)
    {
        // 停止视频流
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd_, VIDIOC_STREAMOFF, &type);

        close(fd_);
    }
}

void CameraPublisher::configure_device()
{
    struct v4l2_format fmt;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    // 获取当前格式
    if (ioctl(fd_, VIDIOC_G_FMT, &fmt) == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to get format");
        return;
    }

    // 设置图像格式为 MJPEG
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;  // 设置为 MJPEG 格式
    fmt.fmt.pix.width = width_;  // 设置为640
    fmt.fmt.pix.height = height_; // 设置为480
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    // 设置新的格式
    if (ioctl(fd_, VIDIOC_S_FMT, &fmt) == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to set format to MJPEG");
        return;
    }

    // 请求缓冲区
    struct v4l2_requestbuffers req;
    req.count = buffer_count_;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd_, VIDIOC_REQBUFS, &req) == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to request buffers");
        return;
    }

    // 映射缓冲区
    buffers_.resize(buffer_count_);
    for (unsigned int i = 0; i < buffer_count_; ++i)
    {
        struct v4l2_buffer buf;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        // 查询缓冲区信息
        if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to query buffer");
            return;
        }

        // 映射内存：通过文件描述符获取内存
        buffers_[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, buf.m.fd, 0);
        buffers_[i].length = buf.length;

        /*if (buffers_[i].start == MAP_FAILED)
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to map buffer memory");
            return;
        }*/
        if (buffers_[i].start == MAP_FAILED)
{
    perror("mmap failed");
    RCLCPP_ERROR(this->get_logger(), "Unable to map buffer memory: %s", strerror(errno));
    return;
}

    }

    // 启动视频流
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMON, &type) == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to start streaming");
        return;
    }
}



void CameraPublisher::capture_and_publish()
{
    struct v4l2_buffer buf;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    
    if (ioctl(fd_, VIDIOC_DQBUF, &buf) == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to dequeue buffer");
        return;
    }

    // 获取图像数据
    unsigned char* image_data = static_cast<unsigned char*>(buffers_[buf.index].start);

    // 将图像数据发布到 ROS 话题
    sensor_msgs::msg::Image msg;
    msg.header.stamp = this->get_clock()->now();
    msg.height = height_;  // 修改为480
    msg.width = width_;    // 修改为640
    msg.encoding = "nv12"; // 使用 NV12 编码
    msg.is_bigendian = false;
    msg.step = width_;     // 每行的字节数
    msg.data.assign(image_data, image_data + buf.bytesused);
    image_pub_->publish(msg);

    // 重用缓冲区
    if (ioctl(fd_, VIDIOC_QBUF, &buf) == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to queue buffer");
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}
