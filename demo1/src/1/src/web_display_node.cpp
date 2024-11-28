// camera_streamer/src/web_display_node.cpp

#include <rclcpp/rclcpp.hpp>
#include "camera_streamer/msg/shared_image.hpp"
#include <fcntl.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>
#include <vector>
#include <jpeglib.h>

#define SHM_NAME "/camera_image_shm"  // 共享内存名称
#define SEM_NAME "/camera_image_sem"  // 信号量名称
#define PORT 8000                     // 服务器端口

class WebDisplayNode : public rclcpp::Node
{
public:
  WebDisplayNode()
  : Node("web_display_node"), image_ready_(false)
  {
    // 订阅自定义消息
    subscription_ = this->create_subscription<camera_streamer::msg::SharedImage>(
      "shared_image", 10, std::bind(&WebDisplayNode::topic_callback, this, std::placeholders::_1));

    // 打开共享内存
    shm_fd_ = shm_open(SHM_NAME, O_RDONLY, 0666);
    if (shm_fd_ == -1) {
      RCLCPP_ERROR(this->get_logger(), "无法打开共享内存: %s", strerror(errno));
      rclcpp::shutdown();
      return;
    }

    // 获取共享内存大小
    struct stat shm_stat;
    if (fstat(shm_fd_, &shm_stat) == -1) {
      RCLCPP_ERROR(this->get_logger(), "无法获取共享内存大小: %s", strerror(errno));
      rclcpp::shutdown();
      return;
    }
    shm_size_ = shm_stat.st_size;

    // 映射共享内存
    shm_addr_ = mmap(NULL, shm_size_, PROT_READ, MAP_SHARED, shm_fd_, 0);
    if (shm_addr_ == MAP_FAILED) {
      RCLCPP_ERROR(this->get_logger(), "映射共享内存失败: %s", strerror(errno));
      rclcpp::shutdown();
      return;
    }

    // 打开信号量
    sem_ = sem_open(SEM_NAME, 0);
    if (sem_ == SEM_FAILED) {
      RCLCPP_ERROR(this->get_logger(), "打开信号量失败: %s", strerror(errno));
      rclcpp::shutdown();
      return;
    }

    // 启动 HTTP 服务器线程
    server_thread_ = std::thread(&WebDisplayNode::start_server, this);
  }

  ~WebDisplayNode()
  {
    // 释放资源
    munmap(shm_addr_, shm_size_);
    close(shm_fd_);

    sem_close(sem_);

    if (server_thread_.joinable()) {
      server_thread_.join();
    }
  }

private:
  void topic_callback(const camera_streamer::msg::SharedImage::SharedPtr msg)
  {
    // 存储图像元数据
    width_ = msg->width;
    height_ = msg->height;
    encoding_ = msg->encoding;
    timestamp_ = msg->timestamp;

    image_ready_ = true;
  }

  void start_server()
  {
    int server_fd, new_socket;
    struct sockaddr_in address;
    int addrlen = sizeof(address);

    // 创建套接字
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
      RCLCPP_ERROR(this->get_logger(), "Socket 创建失败: %s", strerror(errno));
      return;
    }

    // 绑定端口
    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
      RCLCPP_ERROR(this->get_logger(), "设置套接字选项失败: %s", strerror(errno));
      close(server_fd);
      return;
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "绑定端口失败: %s", strerror(errno));
      close(server_fd);
      return;
    }

    if (listen(server_fd, 3) < 0) {
      RCLCPP_ERROR(this->get_logger(), "监听失败: %s", strerror(errno));
      close(server_fd);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "HTTP 服务器已启动，端口 %d", PORT);

    while (rclcpp::ok()) {
      if ((new_socket = accept(server_fd, (struct sockaddr *)&address,
                               (socklen_t *)&addrlen)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "接受连接失败: %s", strerror(errno));
        continue;
      }

      // 启动新线程处理客户端连接
      std::thread(&WebDisplayNode::handle_client, this, new_socket).detach();
    }

    close(server_fd);
  }

  void handle_client(int client_socket)
  {
    // 读取 HTTP 请求
    char buffer[1024] = {0};
    read(client_socket, buffer, sizeof(buffer));

    // 简单检查 GET 请求
    if (strncmp(buffer, "GET /", 5) == 0) {
      // 发送 MJPEG 流的 HTTP 头
      const char *header =
          "HTTP/1.0 200 OK\r\n"
          "Cache-Control: no-cache\r\n"
          "Cache-Control: private\r\n"
          "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";

      send(client_socket, header, strlen(header), 0);

      while (rclcpp::ok()) {
        if (image_ready_) {
          image_ready_ = false;

          // 加锁
          sem_wait(sem_);

          // 从共享内存复制图像数据
          std::vector<uint8_t> nv12_data((uint8_t *)shm_addr_, (uint8_t *)shm_addr_ + shm_size_);

          // 解锁
          sem_post(sem_);

          // 转换 NV12 到 JPEG
          std::vector<uint8_t> jpeg_data;
          if (nv12_to_jpeg(nv12_data, width_, height_, jpeg_data)) {
            // 发送 JPEG 帧
            std::string boundary = "--frame\r\n";
            std::string content_type = "Content-Type: image/jpeg\r\n";
            std::string content_length = "Content-Length: " + std::to_string(jpeg_data.size()) + "\r\n\r\n";

            send(client_socket, boundary.c_str(), boundary.size(), 0);
            send(client_socket, content_type.c_str(), content_type.size(), 0);
            send(client_socket, content_length.c_str(), content_length.size(), 0);
            send(client_socket, (char *)jpeg_data.data(), jpeg_data.size(), 0);
            send(client_socket, "\r\n", 2, 0);
          }
        }
        // 睡眠以避免忙等待
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
      }
    }

    close(client_socket);
  }

  bool nv12_to_jpeg(const std::vector<uint8_t> &nv12_data, int width, int height, std::vector<uint8_t> &jpeg_data)
  {
    // 将 NV12 转换为 RGB24
    std::vector<uint8_t> rgb_data(width * height * 3);
    nv12_to_rgb24(nv12_data.data(), rgb_data.data(), width, height);

    // 压缩 RGB24 到 JPEG
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    JSAMPROW row_pointer[1];
    unsigned char *outbuffer = NULL;
    unsigned long outsize = 0;

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);

    jpeg_mem_dest(&cinfo, &outbuffer, &outsize);

    cinfo.image_width = width;
    cinfo.image_height = height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults(&cinfo);
    jpeg_start_compress(&cinfo, TRUE);

    while (cinfo.next_scanline < cinfo.image_height) {
      row_pointer[0] = &rgb_data[cinfo.next_scanline * width * 3];
      jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    // 复制 JPEG 数据
    jpeg_data.assign(outbuffer, outbuffer + outsize);
    free(outbuffer);

    return true;
  }

  void nv12_to_rgb24(const uint8_t *nv12, uint8_t *rgb, int width, int height)
  {
    int frameSize = width * height;
    const uint8_t *yPlane = nv12;
    const uint8_t *uvPlane = nv12 + frameSize;

    for (int j = 0; j < height; j++) {
      for (int i = 0; i < width; i++) {
        int y = yPlane[j * width + i];
        int uv_index = (j / 2) * width + (i & ~1);
        int u = uvPlane[uv_index] - 128;
        int v = uvPlane[uv_index + 1] - 128;

        int r = y + (1.370705 * v);
        int g = y - (0.337633 * u) - (0.698001 * v);
        int b = y + (1.732446 * u);

        r = std::min(255, std::max(0, r));
        g = std::min(255, std::max(0, g));
        b = std::min(255, std::max(0, b));

        rgb[(j * width + i) * 3 + 0] = r;
        rgb[(j * width + i) * 3 + 1] = g;
        rgb[(j * width + i) * 3 + 2] = b;
      }
    }
  }

  rclcpp::Subscription<camera_streamer::msg::SharedImage>::SharedPtr subscription_;
  int shm_fd_;
  void *shm_addr_;
  size_t shm_size_;
  sem_t *sem_;
  std::thread server_thread_;
  bool image_ready_;
  uint32_t width_;
  uint32_t height_;
  std::string encoding_;
  rclcpp::Time timestamp_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WebDisplayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}