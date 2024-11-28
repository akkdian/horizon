// camera_streamer/src/camera_publisher.cpp

#include <rclcpp/rclcpp.hpp>
#include "camera_streamer/msg/shared_image.hpp"
#include <linux/videodev2.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <semaphore.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>

#define SHM_NAME "/camera_image_shm"  // 共享内存名称
#define SEM_NAME "/camera_image_sem"  // 信号量名称
#define IMAGE_WIDTH 1280              // 图像宽度
#define IMAGE_HEIGHT 720              // 图像高度

using namespace std::chrono_literals;

class CameraPublisher : public rclcpp::Node
{
public:
  CameraPublisher()
  : Node("camera_publisher")
  {
    // 创建自定义消息的发布者
    publisher_ = this->create_publisher<camera_streamer::msg::SharedImage>("shared_image", 10);

    // 打开摄像头设备
    video_fd_ = open("/dev/video0", O_RDWR);
    if (video_fd_ == -1) {
      RCLCPP_ERROR(this->get_logger(), "无法打开摄像头设备: %s", strerror(errno));
      rclcpp::shutdown();
      return;
    }

    // 初始化摄像头设备
    if (!init_video_device()) {
      RCLCPP_ERROR(this->get_logger(), "初始化摄像头设备失败");
      rclcpp::shutdown();
      return;
    }

    // 设置共享内存
    if (!setup_shared_memory()) {
      RCLCPP_ERROR(this->get_logger(), "设置共享内存失败");
      rclcpp::shutdown();
      return;
    }

    // 开始摄像头采集
    if (!start_capturing()) {
      RCLCPP_ERROR(this->get_logger(), "开始采集失败");
      rclcpp::shutdown();
      return;
    }

    // 创建定时器，定期采集图像并发布消息
    timer_ = this->create_wall_timer(30ms, std::bind(&CameraPublisher::timer_callback, this));
  }

  ~CameraPublisher()
  {
    // 停止采集并释放资源
    stop_capturing();
    uninit_video_device();
    close(video_fd_);

    munmap(shm_addr_, shm_size_);
    close(shm_fd_);
    shm_unlink(SHM_NAME);

    sem_close(sem_);
    sem_unlink(SEM_NAME);
  }

private:
  bool init_video_device()
  {
    // 设置视频格式
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = IMAGE_WIDTH;
    fmt.fmt.pix.height = IMAGE_HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(video_fd_, VIDIOC_S_FMT, &fmt) == -1) {
      RCLCPP_ERROR(this->get_logger(), "设置视频格式失败: %s", strerror(errno));
      return false;
    }

    // 请求缓冲区
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(video_fd_, VIDIOC_REQBUFS, &req) == -1) {
      RCLCPP_ERROR(this->get_logger(), "请求缓冲区失败: %s", strerror(errno));
      return false;
    }

    // 查询缓冲区
    memset(&buf_, 0, sizeof(buf_));
    buf_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf_.memory = V4L2_MEMORY_MMAP;
    buf_.index = 0;

    if (ioctl(video_fd_, VIDIOC_QUERYBUF, &buf_) == -1) {
      RCLCPP_ERROR(this->get_logger(), "查询缓冲区失败: %s", strerror(errno));
      return false;
    }

    // 映射缓冲区
    buffer_start_ = mmap(NULL, buf_.length, PROT_READ | PROT_WRITE, MAP_SHARED, video_fd_, buf_.m.offset);

    if (buffer_start_ == MAP_FAILED) {
      RCLCPP_ERROR(this->get_logger(), "映射缓冲区失败: %s", strerror(errno));
      return false;
    }

    return true;
  }

  bool setup_shared_memory()
  {
    // 打开共享内存
    shm_fd_ = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd_ == -1) {
      RCLCPP_ERROR(this->get_logger(), "打开共享内存失败: %s", strerror(errno));
      return false;
    }

    // 设置共享内存大小
    shm_size_ = buf_.length;
    if (ftruncate(shm_fd_, shm_size_) == -1) {
      RCLCPP_ERROR(this->get_logger(), "设置共享内存大小失败: %s", strerror(errno));
      return false;
    }

    // 映射共享内存
    shm_addr_ = mmap(NULL, shm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
    if (shm_addr_ == MAP_FAILED) {
      RCLCPP_ERROR(this->get_logger(), "映射共享内存失败: %s", strerror(errno));
      return false;
    }

    // 初始化信号量
    sem_ = sem_open(SEM_NAME, O_CREAT, 0666, 1);
    if (sem_ == SEM_FAILED) {
      RCLCPP_ERROR(this->get_logger(), "打开信号量失败: %s", strerror(errno));
      return false;
    }

    return true;
  }

  bool start_capturing()
  {
    // 入队缓冲区
    if (ioctl(video_fd_, VIDIOC_QBUF, &buf_) == -1) {
      RCLCPP_ERROR(this->get_logger(), "缓冲区入队失败: %s", strerror(errno));
      return false;
    }

    // 开始视频流
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(video_fd_, VIDIOC_STREAMON, &type) == -1) {
      RCLCPP_ERROR(this->get_logger(), "启动视频流失败: %s", strerror(errno));
      return false;
    }

    return true;
  }

  void stop_capturing()
  {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(video_fd_, VIDIOC_STREAMOFF, &type);
  }

  void uninit_video_device()
  {
    munmap(buffer_start_, buf_.length);
  }

  void timer_callback()
  {
    // 取出缓冲区
    if (ioctl(video_fd_, VIDIOC_DQBUF, &buf_) == -1) {
      RCLCPP_ERROR(this->get_logger(), "取出缓冲区失败: %s", strerror(errno));
      return;
    }

    // 加锁
    sem_wait(sem_);

    // 将图像数据复制到共享内存
    memcpy(shm_addr_, buffer_start_, buf_.length);

    // 解锁
    sem_post(sem_);

    // 发布自定义消息
    auto msg = camera_streamer::msg::SharedImage();
    msg.shm_name = SHM_NAME;
    msg.width = IMAGE_WIDTH;
    msg.height = IMAGE_HEIGHT;
    msg.encoding = "nv12";
    msg.timestamp = this->now();

    publisher_->publish(msg);

    // 重新入队缓冲区
    if (ioctl(video_fd_, VIDIOC_QBUF, &buf_) == -1) {
      RCLCPP_ERROR(this->get_logger(), "缓冲区重新入队失败: %s", strerror(errno));
      return;
    }
  }

  int video_fd_;
  void *buffer_start_;
  struct v4l2_buffer buf_;
  rclcpp::Publisher<camera_streamer::msg::SharedImage>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  int shm_fd_;
  void *shm_addr_;
  size_t shm_size_;
  sem_t *sem_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
