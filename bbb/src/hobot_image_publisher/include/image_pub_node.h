// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <vector>

#include "hbm_img_msgs/msg/hbm_h26_x_frame.hpp"
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include "img_msgs/msg/h26_x_frame.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#ifdef __cplusplus
extern "C" {
#endif
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libavutil/log.h>
#include <libavutil/opt.h>
#include <libavutil/time.h>
#include <libswresample/swresample.h>
#ifdef __cplusplus
}
#endif

#ifndef INCLUDE_IMAGE_PUB_NODE_H_
#define INCLUDE_IMAGE_PUB_NODE_H_

struct ImageCache {
  std::string image_;
  cv::Mat nv12_mat;
  std::vector<uint8_t> jpeg;
  uint8_t *img_data = nullptr;
  int32_t width = 0;
  int32_t height = 0;
  int32_t data_len = 0;
  int32_t count_ = 0;
};

struct VideoInfo {
  std::string video_file_;    // 视频文件名
  std::string stream_codec_;  // 视频流编码格式 h264/h265
  AVFormatContext *fmt_ctx = nullptr;
  int video_stream_index = 0;  //文件中视频流index
  float fps;                   // 视频发布帧率
  int32_t width = 0;
  int32_t height = 0;
  uint32_t count_ = 0;  //发布的视频帧count
};

/**
 * 根据传入的ffmpeg错误码获取错误信息
 * @param[in] errnum: ffmpeg的错误码
 * @return 错误信息字符串
 */
inline char *av_err2str_(int errnum) {
  char tmp[AV_ERROR_MAX_STRING_SIZE] = {0};
  return av_make_error_string(tmp, AV_ERROR_MAX_STRING_SIZE, errnum);
}

class PubNode : public rclcpp::Node {
 public:
  PubNode(const std::string &node_name,
          const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~PubNode() override;

 private:
  std::vector<std::string> data_list_;

  std::string image_format_;
  std::string msg_pub_topic_name_;
  std::string hbmem_pub_topic_ = "/hbmem_img";
  std::string ros_pub_topic_ = "/image_raw";
  std::string image_source_ = "./config/test1.jpg";

  int32_t source_image_w_ = 0;
  int32_t source_image_h_ = 0;
  int32_t output_image_w_ = 0;
  int32_t output_image_h_ = 0;
  int32_t fps_ = 10;

  bool is_shared_mem_ = true;
  bool is_loop_ = true;
  bool is_pub_video = false;
  uint32_t pub_index = 0;
  // 是否直接发布jpeg/jpg/png格式的压缩图片
  bool is_compressed_img_pub_ = false;

  ImageCache image_cache_;  // 保存当前发布的图片信息
  VideoInfo video_info_;  // 保存当前发布的视频编码参数，fps，分辨率等

  rclcpp::TimerBase::SharedPtr timer_;

  // 用于shared_mem方式发布图片
  rclcpp::Publisher<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr
      publisher_hbmem_ = nullptr;
  // 用于shared_mem方式发布h26x视频流
  rclcpp::Publisher<hbm_img_msgs::msg::HbmH26XFrame>::SharedPtr
      publisher_hbmem_h26x_ = nullptr;

  //用于ros方式发布图片
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ros_publisher_ =
      nullptr;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr ros_publisher_compressed_ =
      nullptr;

  //用于ros方式发布h26x视频流
  rclcpp::Publisher<img_msgs::msg::H26XFrame>::SharedPtr ros_publisher_h26x_ =
      nullptr;

  // 以固定帧率发布图片
  void timer_callback();

  // 设置pub_index，用于确定循环发布时，多个图片/视频中的哪一个
  void set_pub_index();

  // 获取video信息，fps，分辨率，视频编码格式等
  // 输入参数为video文件的路径，获取的文件信息保存在类成员video_info_中
  // 成功返回0，失败返回-1
  int get_video_info(const std::string &video_file);

  //发布h264、h265类型的topic
  void pub_h26x();

  // 从MP4文件中提取并拼接h264视频流，然后发布h264类型的topic
  void pub_H264FromMP4();
};

#endif  // INCLUDE_IMAGE_PUB_NODE_H_
