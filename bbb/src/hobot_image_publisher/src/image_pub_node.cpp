#include "include/image_pub_node.h"

#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcpputils/env.hpp"
#include "rcutils/env.h"

#define MAX_SIZE 1920 * 1080 * 3

#define MAX_HBMH26XFRAME 512000

// h264 起始码
uint8_t startCode[4] = {0x00, 0x00, 0x00, 0x01};

// hobot_image_publisher支持发布的格式，包括图片格式和视频格式
std::vector<std::string> format_list = {
    {"jpeg", "jpg", "nv12", "png", "mp4", "h264", "h265"}};

// hobot_image_publisher支持发布的视频格式
std::vector<std::string> video_list = {{"mp4", "h264", "h265"}};

/**
 * @brief 检查输入的格式format是否是hobot_image_publisher支持的格式
 * @param[in] format ：需要判断的格式
 * @param[in] fmtTypes ：hobot_image_publisher支持的格式集合
 * @return true：format是hobot_image_publisher支持的格式
 * @return false format不是hobot_image_publisher支持的格式
 */
static bool CheckFormat(const char *format,
                        const std::vector<std::string> &fmtTypes) {
  for (uint32_t nIdx = 0; nIdx < fmtTypes.size(); ++nIdx) {
    if (0 == strcmp(format, fmtTypes[nIdx].c_str())) return true;
  }
  return false;
}

/**
 * @brief Get the String From Vec object
 *        将hobot_image_publisher支持的格式拼接成一个字符串，不同格式之间用字符'/'隔开
 *    例如当前支持的格式{"jpeg", "jpg", "nv12", "png", "mp4", "h264", "h265"}，
 *    返回结果为jpeg/jpg/nv12/png/mp4/h264/h265
 * @param fmtTypes ：hobot_image_publisher支持的格式集合
 * @return std::string 拼接后的string
 */
static std::string SpliceStringFromVec(
    const std::vector<std::string> &fmtTypes) {
  std::string str = "";
  for (uint32_t i = 0; i < fmtTypes.size(); ++i) {
    str.append(fmtTypes[i]);
    if (i != fmtTypes.size() - 1) {
      str.append("/");
    }
  }
  return str;
}

int32_t BGRToNv12(const cv::Mat &bgr_mat, cv::Mat &img_nv12) {
  auto height = bgr_mat.rows;
  auto width = bgr_mat.cols;
  if (height % 2 || width % 2) {
    RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
                 "Image height and width must aligned by 2\n"
                 "height: %d \nwidth: %d",
                 height,
                 width);
    return -1;
  }
  cv::Mat yuv_mat;
  cv::cvtColor(bgr_mat, yuv_mat, cv::COLOR_BGR2YUV_I420);
  if (yuv_mat.data == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
                 "yuv_mat.data is null pointer");
    return -1;
  }

  auto *yuv = yuv_mat.ptr<uint8_t>();
  if (yuv == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
                 "yuv_mat.data is null pointer");
    return -1;
  }
  img_nv12 = cv::Mat(height * 3 / 2, width, CV_8UC1);
  auto *ynv12 = img_nv12.ptr<uint8_t>();

  int32_t uv_height = height / 2;
  int32_t uv_width = width / 2;

  int32_t y_size = height * width;
  memcpy(ynv12, yuv, y_size);

  uint8_t *nv12 = ynv12 + y_size;
  uint8_t *u_data = yuv + y_size;
  uint8_t *v_data = u_data + uv_height * uv_width;

  for (int32_t i = 0; i < uv_width * uv_height; i++) {
    *nv12++ = *u_data++;
    *nv12++ = *v_data++;
  }
  return 0;
}

int32_t checkPathType(const std::string &source) {
  struct stat image_source_stat;
  int32_t ret = 0;
  if (stat(source.c_str(), &image_source_stat) == 0) {
    if (image_source_stat.st_mode & S_IFDIR) {
      RCLCPP_INFO(rclcpp::get_logger("image_pub_node"),
                  "Your path is a folder");
    } else if (image_source_stat.st_mode & S_IFREG) {
      RCLCPP_INFO(rclcpp::get_logger("image_pub_node"), "Your path is a file");
      ret = 1;
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
                 "Source: %s not exist!",
                 source.c_str());
    rclcpp::shutdown();
    return -1;
  }
  return ret;
}

void resizeImage(cv::Mat &in_mat,
                 cv::Mat &out_mat,
                 const int32_t &out_height,
                 const int32_t &out_width,
                 const int32_t &in_width,
                 const int32_t &in_height) {
  float ratio_w = 0.f;
  float ratio_h = 0.f;
  float dst_ratio = 0.f;
  uint32_t resized_width = 0;
  uint32_t resized_height = 0;
  if (in_width > out_width || in_height > out_height) {
    ratio_w = static_cast<float>(in_width) / static_cast<float>(out_width);
    ratio_h = static_cast<float>(in_height) / static_cast<float>(out_height);
    dst_ratio = std::max(ratio_w, ratio_h);
    resized_width = static_cast<float>(in_width) / dst_ratio;
    resized_height = static_cast<float>(in_height) / dst_ratio;
    cv::resize(in_mat, in_mat, cv::Size(resized_width, resized_height));
  } else {
    ratio_w = static_cast<float>(out_width) / static_cast<float>(in_width);
    ratio_h = static_cast<float>(out_height) / static_cast<float>(in_height);
    dst_ratio = std::min(ratio_w, ratio_h);
    resized_width = static_cast<float>(in_width) * dst_ratio;
    resized_height = static_cast<float>(in_height) * dst_ratio;
    cv::resize(in_mat, in_mat, cv::Size(resized_width, resized_height));
  }
  in_mat.copyTo(out_mat(cv::Rect((out_width - in_mat.cols) / 2,
                                 (out_height - in_mat.rows) / 2,
                                 in_mat.cols,
                                 in_mat.rows)));
}

void processImage(ImageCache &image_cache,
                  const std::string &image_source,
                  const int32_t &output_image_w,
                  const int32_t &output_image_h,
                  const int32_t &source_image_w,
                  const int32_t &source_image_h,
                  const std::string &image_format,
                  bool is_compressed_img_pub) {
  image_cache.image_ = image_source;
  if (access(image_cache.image_.c_str(), R_OK) == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
                 "Image: %s not exist!",
                 image_source.c_str());
    rclcpp::shutdown();
    return;
  }
  cv::Mat bgr_mat;
  cv::Mat nv12_tmp;

  // 获取图片
  if (image_format == "nv12") {
    std::ifstream ifs(image_cache.image_, std::ios::in | std::ios::binary);
    if (!ifs) {
      RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
                   "Image: %s not exist!",
                   image_source.c_str());
      rclcpp::shutdown();
      return;
    }
    ifs.seekg(0, std::ios::end);
    int32_t len = ifs.tellg();
    if (len != source_image_h * source_image_w * 3 / 2) {
      RCLCPP_ERROR(
          rclcpp::get_logger("image_pub_node"),
          "Parameters: source_image_w and source_image_h are set incorrectly!\n"
          "The length of the nv12 file should be equal to source_image_h * "
          "source_image_w * 3 / 2 \n"
          "Length of %s: %d \nsource_image_h: %d \nsource_image_w: %d",
          image_source.c_str(),
          len,
          source_image_h,
          source_image_w);
      rclcpp::shutdown();
      return;
    }
    ifs.seekg(0, std::ios::beg);
    nv12_tmp = cv::Mat(source_image_h * 3 / 2, source_image_w, CV_8UC1);
    auto *nv12_data_ptr = reinterpret_cast<char *>(nv12_tmp.ptr<uint8_t>());
    ifs.read(nv12_data_ptr, len);
    ifs.close();
  } else {
    bgr_mat = cv::imread(image_cache.image_, cv::IMREAD_COLOR);
  }

  // 根据配置参数获取分辨率
  int32_t ori_width = (image_format == "nv12") ? source_image_w : bgr_mat.cols;
  int32_t ori_height = (image_format == "nv12") ? source_image_h : bgr_mat.rows;
  int32_t pad_width = (output_image_w == 0) ? ori_width : output_image_w;
  int32_t pad_height = (output_image_h == 0) ? ori_height : output_image_h;
  if ((pad_width < 0) || (pad_height < 0)) {
    RCLCPP_ERROR(
        rclcpp::get_logger("image_pub_node"),
        "Parameters: output_image_w and output_image_h are set incorrectly! "
        "output_image_w and output_image_h should not be negative!\n"
        "output_image_w :%d \noutput_image_h :%d",
        output_image_w,
        output_image_h);
    rclcpp::shutdown();
    return;
  }
  // 根据配置参数改变图片分辨率以及格式转换
  cv::Mat pad_frame(pad_height, pad_width, CV_8UC3, cv::Scalar::all(0));
  if (ori_width != pad_width || ori_height != pad_height) {
    if (image_format == "nv12") {
      cv::cvtColor(nv12_tmp, bgr_mat, cv::COLOR_YUV2BGR_NV12);
    }
    resizeImage(
        bgr_mat, pad_frame, pad_height, pad_width, ori_width, ori_height);
  } else {
    pad_frame = bgr_mat;
  }
  if (is_compressed_img_pub) {
    // 使用opencv的imencode接口将mat转成vector，获取图片size
    std::vector<int> param;
    imencode(".jpg", pad_frame, image_cache.jpeg, param);
    image_cache.img_data = image_cache.jpeg.data();
    image_cache.data_len = image_cache.jpeg.size();
  } else {
    cv::Mat &nv12_mat = image_cache.nv12_mat;
    if (image_format == "nv12" &&
        (ori_width == pad_width && ori_height == pad_height)) {
      nv12_mat = nv12_tmp;
    } else {
      auto ret = BGRToNv12(pad_frame, nv12_mat);
      if (ret) {
        RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
                    "Image: %s get nv12 image failed",
                    image_source.c_str());
        rclcpp::shutdown();
        return;
      }
    }
    image_cache.img_data = image_cache.nv12_mat.data;
    image_cache.data_len = pad_width * pad_height * 3 / 2;
  }
  image_cache.width = pad_width;
  image_cache.height = pad_height;
}

// 获取video信息，fps，分辨率，视频编码格式等
int PubNode::get_video_info(const std::string &file) {
  if (access(file.c_str(), R_OK) == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
                 "Video file: %s not exist!",
                 file.c_str());
    exit(-1);
    return -1;
  }

  // 打开video文件，获取context
  int ret =
      avformat_open_input(&(video_info_.fmt_ctx), file.c_str(), NULL, NULL);
  if (ret < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
                 "open file failed: %s, %s",
                 file.c_str(),
                 av_err2str_(ret));
    exit(-1);
    return -1;
  }
  video_info_.video_file_ = file;
  RCLCPP_INFO(rclcpp::get_logger("image_pub_node"),
              "open video file: %s",
              file.c_str());
  // 查找stream信息
  ret = avformat_find_stream_info(video_info_.fmt_ctx, NULL);
  if (ret < 0) {
    RCLCPP_WARN(rclcpp::get_logger("image_pub_node"),
                "Cannot find stream info: %s. [AV_ERROR]: %s",
                file.c_str(),
                av_err2str_(ret));
  }

  // 寻找video中视频流
  video_info_.video_stream_index = av_find_best_stream(
      video_info_.fmt_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
  if (ret < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
                 "Cannot find a video stream in the input file: %s",
                 file.c_str());
    exit(-1);
    return -1;
  }

  // 获取视频帧率fps
  const AVStream *stream =
      video_info_.fmt_ctx->streams[video_info_.video_stream_index];
  float fps_temp = av_q2d(stream->avg_frame_rate);
  if (fps_temp != 0.0f) {
    video_info_.fps = fps_temp;
  } else {  // 获取视频帧率失败，采用传入的fps值
    RCLCPP_WARN(rclcpp::get_logger("image_pub_node"),
                "get fps failed from stream, use parameter fps: %d instead!",
                fps_);
    video_info_.fps = fps_;
  }
  RCLCPP_INFO(rclcpp::get_logger("image_pub_node"),
              "video index: %d, fps: %f",
              video_info_.video_stream_index,
              video_info_.fps);

  AVCodecContext *avctx = avcodec_alloc_context3(NULL);
  ret = avcodec_parameters_to_context(avctx, stream->codecpar);
  // 获取视频编码类型(h264/h265)
  video_info_.stream_codec_ = avcodec_get_name(avctx->codec_id);
  if (video_info_.stream_codec_ == "hevc") {
    // hevc与h265属于同一种格式，只是名称不同
    video_info_.stream_codec_ = "h265";
  }

  if (image_format_ == "mp4" && video_info_.stream_codec_ != "h264") {
    RCLCPP_ERROR(
        rclcpp::get_logger("image_pub_node"),
        "stream codec is %s but parameter image_format is mp4! "
        "The video stream encoding format of mp4 files should be h264!",
        video_info_.stream_codec_.c_str());
    exit(-1);
    return -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("image_pub_node"),
              "stream codec:%s",
              video_info_.stream_codec_.c_str());

  // 获取视频的分辨率
  if (pub_index != 0) {
    if (avctx->width != video_info_.width ||
        avctx->height != video_info_.height) {
      // 视频分辨率发生了变化
      RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
                   "%s resolution is %d x %d, the previous resolution is %d x "
                   "%d. Please publish videos with same resolution!\n",
                   file.c_str(),
                   avctx->width,
                   avctx->height,
                   video_info_.width,
                   video_info_.height);
      exit(-1);
      return -1;
    }
  }
  video_info_.width = avctx->width;
  video_info_.height = avctx->height;
  RCLCPP_INFO(rclcpp::get_logger("image_pub_node"),
              "resolution: %d x %d\n",
              video_info_.width,
              video_info_.height);
  avcodec_free_context(&avctx);
  return 0;
}

// 设置pub_index，用于确定循环发布时，多个图片/视频中的哪一个
void PubNode::set_pub_index() {
  pub_index++;
  if (is_loop_ == true) {
    if (data_list_.size() != 0) {
      pub_index = pub_index % data_list_.size();
    }
  } else {
    if (data_list_.size() != 0) {
      if (pub_index == data_list_.size()) {
        rclcpp::shutdown();
        return;
      }
    } else {
      rclcpp::shutdown();
      return;
    }
  }
}

PubNode::PubNode(const std::string &node_name,
                 const rclcpp::NodeOptions &options)
    : rclcpp::Node(node_name, options) {
  this->declare_parameter<int32_t>("source_image_w", source_image_w_);
  this->declare_parameter<int32_t>("source_image_h", source_image_h_);
  this->declare_parameter<int32_t>("output_image_w", output_image_w_);
  this->declare_parameter<int32_t>("output_image_h", output_image_h_);
  this->declare_parameter<int32_t>("fps", fps_);
  this->declare_parameter<bool>("is_shared_mem", is_shared_mem_);
  this->declare_parameter<bool>("is_loop", is_loop_);
  this->declare_parameter<bool>("is_compressed_img_pub", is_compressed_img_pub_);
  this->declare_parameter<std::string>("image_source", image_source_);
  this->declare_parameter<std::string>("image_format", image_format_);
  this->declare_parameter<std::string>("msg_pub_topic_name",
                                       msg_pub_topic_name_);

  this->get_parameter<std::string>("image_source", image_source_);
  this->get_parameter<int32_t>("source_image_w", source_image_w_);
  this->get_parameter<int32_t>("source_image_h", source_image_h_);
  this->get_parameter<int32_t>("output_image_w", output_image_w_);
  this->get_parameter<int32_t>("output_image_h", output_image_h_);
  this->get_parameter<int32_t>("fps", fps_);
  this->get_parameter<bool>("is_shared_mem", is_shared_mem_);
  this->get_parameter<bool>("is_loop", is_loop_);
  this->get_parameter<bool>("is_compressed_img_pub", is_compressed_img_pub_);
  this->get_parameter<std::string>("image_format", image_format_);
  this->get_parameter<std::string>("msg_pub_topic_name", msg_pub_topic_name_);

  if (msg_pub_topic_name_.size() == 0) {
    if (is_shared_mem_ == true) {
      msg_pub_topic_name_ = hbmem_pub_topic_;
    } else {
      msg_pub_topic_name_ = ros_pub_topic_;
    }
  }
  RCLCPP_WARN_STREAM(rclcpp::get_logger("image_pub_node"),
    "parameter:"
    << "\nimage_source: " << image_source_
    << "\nsource_image_w: " << source_image_w_
    << "\nsource_image_h: " << source_image_h_
    << "\noutput_image_w: " << output_image_w_
    << "\noutput_image_h: " << output_image_h_
    << "\nfps: " << fps_
    << "\nis_shared_mem: " << is_shared_mem_
    << "\nis_loop: " << is_loop_
    << "\nis_compressed_img_pub: " << is_compressed_img_pub_
    << "\nimage_format: " << image_format_
    << "\nmsg_pub_topic_name: " << msg_pub_topic_name_);

  if (is_shared_mem_ ) {
    std::string ros_zerocopy_env = rcpputils::get_env_var("RMW_FASTRTPS_USE_QOS_FROM_XML");
    if (ros_zerocopy_env.empty()) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
        "Launching with zero-copy, but env of `RMW_FASTRTPS_USE_QOS_FROM_XML` is not set. "
        << "Transporting data without zero-copy!");
    } else {
      if ("1" == ros_zerocopy_env) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Enabling zero-copy");
      } else {
        RCLCPP_ERROR_STREAM(this->get_logger(),
          "env of `RMW_FASTRTPS_USE_QOS_FROM_XML` is [" << ros_zerocopy_env
          << "], which should be set to 1. "
          << "Data transporting without zero-copy!");
      }
    }
  }

  if (image_format_.size() == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
                 "Please add parameter: image_format to your command!\n"
                 "Example: -p image_format:=jpeg/jpg/png/nv12");
    rclcpp::shutdown();
    return;
  } else {
    if ((source_image_w_ == 0 || source_image_h_ == 0) &&
        image_format_ == "nv12") {
      RCLCPP_ERROR(
          rclcpp::get_logger("image_pub_node"),
          "Your image format is nv12. Please add parameters:source_image_w and "
          "source_image_h to your command!\n"
          "Example: -p source_image_w:=<source_image_w>  -p "
          "source_image_h:=<source_image_h>");
      rclcpp::shutdown();
      return;
    }
    if (!CheckFormat(image_format_.c_str(), format_list)) {
      std::stringstream ss;
      std::string format_str = SpliceStringFromVec(format_list);
      ss << "Parameter: image_format setting error! Only " << format_str
         << " is supported\nimage_format: " << image_format_
         << "\nExample: -p image_format:=" << format_str;
      RCLCPP_ERROR(
          rclcpp::get_logger("image_pub_node"), "%s", ss.str().c_str());
      rclcpp::shutdown();
      return;
    }
    if (CheckFormat(image_format_.c_str(), video_list)) {
      is_pub_video = true;
    } else if ("nv12" == image_format_ && is_compressed_img_pub_) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("image_pub_node"),
        "Paras are unmatch! image_format_: " << image_format_
        << ", is_compressed_img_pub: " << is_compressed_img_pub_
        << " is only valid for compressed img and reset as False.");
      rclcpp::shutdown();
      return;
    }
  }

  if (fps_ <= 0 || fps_ > 30) {
    RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
                 "Parameter: fps setting error! fps should be greater than 0 "
                 "and less than 30!\n"
                 "Fps: %d\n",
                 fps_);
    rclcpp::shutdown();
    return;
  }

  int32_t type = checkPathType(image_source_);
  if (type == 1) {
    std::string file_extension =
        image_source_.substr(image_source_.find_last_of('.') + 1);
    if (image_format_ == file_extension) {
      // 路径为图片或视频文件
      if (!is_pub_video) {
        processImage(image_cache_,
                     image_source_,
                     output_image_w_,
                     output_image_h_,
                     source_image_w_,
                     source_image_h_,
                     image_format_,
                     is_compressed_img_pub_);
      }
    } else if (file_extension == "list") {
      // 路径为list文件
      std::string list_tmp;
      std::fstream list_ifs(image_source_);
      assert(list_ifs.is_open());
      while (getline(list_ifs, list_tmp)) {
        std::string list_file_extension =
            list_tmp.substr(list_tmp.find_last_of('.') + 1);
        if (list_file_extension == image_format_) {
          data_list_.push_back(list_tmp);
        }
      }
      list_ifs.close();
      if (data_list_.size() == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
                     "The image with format %s cannot be found in %s",
                     image_format_.c_str(),
                     image_source_.c_str());
        rclcpp::shutdown();
        return;
      }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
                   "There is no matching file in your path\n"
                   "image_source: %s\n"
                   "image_format: %s\n"
                   "You can try changing the image_format to: %s",
                   image_source_.c_str(),
                   image_format_.c_str(),
                   file_extension.c_str());
      rclcpp::shutdown();
      return;
    }
  } else if (type == 0) {
    // 路径为文件夹
    DIR *pDir;
    struct dirent *ptr;
    if (!(pDir = opendir(image_source_.c_str()))) {
      RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
                   "Can not open the dir: %s",
                   image_source_.c_str());
      rclcpp::shutdown();
      return;
    }
    while ((ptr = readdir(pDir)) != 0) {
      std::string file_name(ptr->d_name);
      std::string file_extension =
          file_name.substr(file_name.find_last_of('.') + 1);
      if ((strcmp(ptr->d_name, ".") != 0) && (strcmp(ptr->d_name, "..") != 0) &&
          (image_format_ == file_extension)) {
        data_list_.push_back(image_source_ + "/" + ptr->d_name);
      }
    }
    closedir(pDir);
    if (data_list_.size() == 0) {
      RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
                   "The format %s cannot be found in %s",
                   image_format_.c_str(),
                   image_source_.c_str());
      rclcpp::shutdown();
      return;
    }
  }

  if (is_pub_video) {  // 发布视频
    if (is_shared_mem_ == true) {
      publisher_hbmem_h26x_ =
          this->create_publisher<hbm_img_msgs::msg::HbmH26XFrame>(
              msg_pub_topic_name_, rclcpp::SensorDataQoS());
    } else {
      ros_publisher_h26x_ =
          this->create_publisher<img_msgs::msg::H26XFrame>(msg_pub_topic_name_, rclcpp::SensorDataQoS());
    }
    if (image_format_ == "h264" || image_format_ == "h265") {
      pub_h26x();
    } else if (image_format_ == "mp4") {
      pub_H264FromMP4();
    }
  } else {  // 发布图片
    if (is_shared_mem_ == true) {
      publisher_hbmem_ =
          this->create_publisher<hbm_img_msgs::msg::HbmMsg1080P>(
              msg_pub_topic_name_, rclcpp::SensorDataQoS());
    } else {
      if (is_compressed_img_pub_) {
        ros_publisher_compressed_ =
            this->create_publisher<sensor_msgs::msg::CompressedImage>(msg_pub_topic_name_, 10);
      } else {
        ros_publisher_ =
            this->create_publisher<sensor_msgs::msg::Image>(msg_pub_topic_name_, 10);
      }
    }
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / fps_),
                                     std::bind(&PubNode::timer_callback, this));
  }
}

PubNode::~PubNode() {
  RCLCPP_WARN(rclcpp::get_logger("image_pub_node"), "destructor");
}

void PubNode::timer_callback() {
  if (data_list_.size() != 0) {
    processImage(image_cache_,
                 data_list_[pub_index],
                 output_image_w_,
                 output_image_h_,
                 source_image_w_,
                 source_image_h_,
                 image_format_,
                 is_compressed_img_pub_);
  }
  if (is_shared_mem_) {
    auto loanedMsg = publisher_hbmem_->borrow_loaned_message();
    if (loanedMsg.is_valid()) {
      auto &msg = loanedMsg.get();
      msg.height = image_cache_.height;
      msg.width = image_cache_.width;
      if (is_compressed_img_pub_) {
        memcpy(msg.encoding.data(), "jpeg", strlen("jpeg"));
      } else {
        memcpy(msg.encoding.data(), "nv12", strlen("nv12"));
      }
      memcpy(&msg.data[0], image_cache_.img_data, image_cache_.data_len);
      struct timespec time_start = {0, 0};
      clock_gettime(CLOCK_REALTIME, &time_start);
      msg.time_stamp.sec = time_start.tv_sec;
      msg.time_stamp.nanosec = time_start.tv_nsec;
      msg.index = ++image_cache_.count_;
      msg.data_size = image_cache_.data_len;
      RCLCPP_INFO(rclcpp::get_logger("image_pub_node"),
                  "Publish hbm image msg,file: %s, encoding: %s, img h: %d, w: "
                  "%d, size: %d, topic: %s",
                  image_cache_.image_.c_str(),
                  msg.encoding.data(),
                  msg.height,
                  msg.width,
                  msg.data_size,
                  msg_pub_topic_name_.data());
      publisher_hbmem_->publish(std::move(loanedMsg));
    }
  } else {
    if (is_compressed_img_pub_) {
      sensor_msgs::msg::CompressedImage::UniquePtr msg(new sensor_msgs::msg::CompressedImage());
      msg->header.stamp = this->now();
      msg->header.frame_id = "default_cam";
      msg->format = "jpeg";
      msg->data.resize(image_cache_.data_len);
      memcpy(&msg->data[0], image_cache_.img_data, image_cache_.data_len);
      RCLCPP_INFO(this->get_logger(),
                  "Publish ros compressed image msg, file: %s, format: %s, topic: %s",
                  image_cache_.image_.c_str(),
                  msg->format.data(),
                  msg_pub_topic_name_.data());
      ros_publisher_compressed_->publish(std::move(msg));
    } else {
      auto msg = sensor_msgs::msg::Image();
      msg.height = image_cache_.height;
      msg.width = image_cache_.width;
      msg.encoding = "nv12";
      msg.data.resize(image_cache_.data_len);
      memcpy(&msg.data[0], image_cache_.img_data, image_cache_.data_len);
      struct timespec time_start = {0, 0};
      clock_gettime(CLOCK_REALTIME, &time_start);
      msg.header.stamp.sec = time_start.tv_sec;
      msg.header.stamp.nanosec = time_start.tv_nsec;
      msg.header.frame_id = std::to_string(++image_cache_.count_);
      RCLCPP_INFO(rclcpp::get_logger("image_pub_node"),
                  "Publish ros image msg, file: %s, encoding: %s, img h: %d, w: "
                  "%d, topic: %s",
                  image_cache_.image_.c_str(),
                  msg.encoding.data(),
                  msg.height,
                  msg.width,
                  msg_pub_topic_name_.data());
      ros_publisher_->publish(msg);
    }
  }
  // 处理循环
  set_pub_index();
}

// 发布h264、h265类型的topic
void PubNode::pub_h26x() {
  do {
    if (data_list_.size() != 0) {
      get_video_info(data_list_[pub_index]);
    } else {
      get_video_info(image_source_);
    }
    AVPacket packet;
    av_init_packet(&packet);
    int packet_index = 0;
    while (av_read_frame(video_info_.fmt_ctx, &packet) >= 0 && rclcpp::ok()) {
      if (is_shared_mem_) {
        if (packet.size > MAX_HBMH26XFRAME) {
          // size大于HbmH26XFrame的最大长度  skip
          RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
                       "packet size is too large, discard it!");
          av_packet_unref(&packet);
          continue;
        }
        auto loanedMsg = publisher_hbmem_h26x_->borrow_loaned_message();
        if (loanedMsg.is_valid()) {
          auto &msg = loanedMsg.get();
          memset((void *)&msg, 0, sizeof(hbm_img_msgs::msg::HbmH26XFrame));
          msg.height = video_info_.height;
          msg.width = video_info_.width;
          memcpy(
              msg.encoding.data(), image_format_.c_str(), image_format_.size());
          msg.index = ++video_info_.count_;
          if (video_info_.count_ == UINT_MAX) {
            video_info_.count_ = 0;
          }
          struct timespec time_start = {0, 0};
          clock_gettime(CLOCK_REALTIME, &time_start);
          msg.dts.sec = time_start.tv_sec;
          msg.dts.nanosec = time_start.tv_nsec;
          msg.pts.sec = time_start.tv_sec;
          msg.pts.nanosec = time_start.tv_nsec;
          ++packet_index;
          if (strcmp("h264", image_format_.c_str()) == 0 || packet_index == 1) {
            msg.data_size = packet.size;
            memcpy(msg.data.data(), packet.data, packet.size);
          } else {
            msg.data_size = packet.size + 1;
            memcpy(msg.data.data() + 1, packet.data, packet.size);
          }
          std::stringstream ss;
          ss << "image_pub_node publish hbm video msg, file: "
             << video_info_.video_file_ << ", topic: " << msg_pub_topic_name_
             << ", encoding: " << msg.encoding.data()
             << ", stamp: " << msg.dts.sec << "." << msg.dts.nanosec
             << ", dLen: " << msg.data_size;
          RCLCPP_INFO(
              rclcpp::get_logger("image_pub_node"), "%s", ss.str().c_str());
          publisher_hbmem_h26x_->publish(std::move(loanedMsg));
          std::this_thread::sleep_for(
              std::chrono::milliseconds((int)(1000.0 / video_info_.fps)));
        }
      } else {
        auto msg = img_msgs::msg::H26XFrame();
        msg.height = video_info_.height;
        msg.width = video_info_.width;
        memcpy(
            msg.encoding.data(), image_format_.c_str(), image_format_.size());
        msg.index = ++video_info_.count_;
        if (video_info_.count_ == UINT_MAX) {
          video_info_.count_ = 0;
        }
        ++packet_index;
        struct timespec time_start = {0, 0};
        clock_gettime(CLOCK_REALTIME, &time_start);
        msg.dts.sec = time_start.tv_sec;
        msg.dts.nanosec = time_start.tv_nsec;
        msg.pts.sec = time_start.tv_sec;
        msg.pts.nanosec = time_start.tv_nsec;
        if (strcmp("h264", (const char *)msg.encoding.data()) == 0) {
          msg.data.resize(packet.size);
          memcpy(msg.data.data(), packet.data, packet.size);
        } else if (packet_index == 1) {
          msg.data.resize(packet.size);
          memcpy(msg.data.data(), packet.data, packet.size);
        } else {
          msg.data.resize(packet.size + 1);
          memset(msg.data.data(), 0, sizeof(uint8_t));
          memcpy(msg.data.data() + 1, packet.data, packet.size);
        }
        std::stringstream ss;
        ss << "image_pub_node publish ros video msg, file: "
           << video_info_.video_file_ << ", topic: " << msg_pub_topic_name_
           << ", encoding: " << msg.encoding.data()
           << ", stamp: " << msg.dts.sec << "." << msg.dts.nanosec
           << ", dLen: " << msg.data.size();
        RCLCPP_INFO(
            rclcpp::get_logger("image_pub_node"), "%s", ss.str().c_str());
        ros_publisher_h26x_->publish(msg);
        std::this_thread::sleep_for(
            std::chrono::milliseconds((int)(1000.0 / video_info_.fps)));
      }
      av_packet_unref(&packet);
    }
    avformat_close_input(&(video_info_.fmt_ctx));
    set_pub_index();
  } while (is_loop_ && rclcpp::ok());
}

// 从MP4文件中提取并拼接h264视频流，然后发布h264类型的topic
void PubNode::pub_H264FromMP4() {
  do {
    if (data_list_.size() != 0) {
      get_video_info(data_list_[pub_index]);
    } else {
      get_video_info(image_source_);
    }
    AVPacket packet;
    bool sendSpsPps = false;
    uint8_t *ex = video_info_.fmt_ctx->streams[video_info_.video_stream_index]
                      ->codecpar->extradata;
    int spsLength = (ex[6] << 8) | ex[7];
    int ppsLength = (ex[8 + spsLength + 1] << 8) | ex[8 + spsLength + 2];
    while (av_read_frame(video_info_.fmt_ctx, &packet) >= 0 && rclcpp::ok()) {
      if (packet.stream_index == video_info_.video_stream_index) {
        if (is_shared_mem_) {
          auto loanedMsg = publisher_hbmem_h26x_->borrow_loaned_message();
          if (loanedMsg.is_valid()) {
            auto &msg = loanedMsg.get();
            memset((void *)&msg, 0, sizeof(hbm_img_msgs::msg::HbmH26XFrame));
            uint8_t *msg_temp = msg.data.data();
            uint8_t *msg_max_addr = msg.data.data() + MAX_HBMH26XFRAME;
            if (!sendSpsPps) {
              // 给SPS拼前4字节起始码
              memcpy(msg_temp, startCode, 4);
              msg_temp += 4;
              // 把SPS数据拼在起始码后面
              memcpy(msg_temp, ex + 8, spsLength);
              msg_temp += spsLength;
              // 给PPS拼前4字节起始码
              memcpy(msg_temp, startCode, 4);
              msg_temp += 4;
              // 把PPS数据拼在起始码后面
              memcpy(msg_temp, ex + 8 + spsLength + 2 + 1, ppsLength);
              msg_temp += ppsLength;
              sendSpsPps = true;
            }
            int nalLength = 0;
            uint8_t *data = packet.data;
            bool isPass = false;
            while (data < packet.data + packet.size) {
              // 取前4字节作为nal的长度
              nalLength =
                  (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
              if (nalLength > 0) {
                memcpy(data, startCode, 4);  // 拼起始码
                if (msg_temp + nalLength + 4 < msg_max_addr) {
                  memcpy(msg_temp, data, nalLength + 4);
                } else {
                  RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
                               "packet size is too large, discard it!");
                  isPass = true;
                  break;
                }
                msg_temp = msg_temp + nalLength + 4;
              }
              data = data + 4 + nalLength;  // 处理data中下一个NALU数据
            }
            if (isPass) {
              av_packet_unref(&packet);
              continue;
            }
            msg.height = video_info_.height;
            msg.width = video_info_.width;
            memcpy(msg.encoding.data(), "h264", strlen("h264"));
            msg.index = ++video_info_.count_;
            if (video_info_.count_ == UINT_MAX) {
              video_info_.count_ = 0;
            }
            struct timespec time_start = {0, 0};
            clock_gettime(CLOCK_REALTIME, &time_start);
            msg.dts.sec = time_start.tv_sec;
            msg.dts.nanosec = time_start.tv_nsec;
            msg.pts.sec = time_start.tv_sec;
            msg.pts.nanosec = time_start.tv_nsec;
            msg.data_size = msg_temp - msg.data.data();
            std::stringstream ss;
            ss << "image_pub_node publish hbm video msg, file: "
               << video_info_.video_file_ << ", topic: " << msg_pub_topic_name_
               << ", encoding: " << msg.encoding.data()
               << ", stamp: " << msg.dts.sec << "." << msg.dts.nanosec
               << ", dLen: " << msg.data_size;
            RCLCPP_INFO(
                rclcpp::get_logger("image_pub_node"), "%s", ss.str().c_str());
            publisher_hbmem_h26x_->publish(std::move(loanedMsg));
            std::this_thread::sleep_for(
                std::chrono::milliseconds((int)(1000.0 / video_info_.fps)));
          }
        } else {
          auto msg = img_msgs::msg::H26XFrame();
          msg.data.resize(MAX_HBMH26XFRAME);
          uint8_t *msg_temp = msg.data.data();
          if (!sendSpsPps) {
            // 给SPS拼前4字节起始码
            memcpy(msg_temp, startCode, 4);
            msg_temp += 4;
            // 把SPS数据拼在起始码后面
            memcpy(msg_temp, ex + 8, spsLength);
            msg_temp += spsLength;
            // 给PPS拼前4字节起始码
            memcpy(msg_temp, startCode, 4);
            msg_temp += 4;
            // 把PPS数据拼在起始码后面
            memcpy(msg_temp, ex + 8 + spsLength + 2 + 1, ppsLength);
            msg_temp += ppsLength;
            sendSpsPps = true;
          }
          int nalLength = 0;
          uint8_t *data = packet.data;
          while (data < packet.data + packet.size) {
            // 取前4字节作为nal的长度
            nalLength =
                (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
            if (nalLength > 0) {
              memcpy(data, startCode, 4);  // 拼起始码
              uint32_t frame_size = msg_temp - msg.data.data();
              if (frame_size + nalLength + 4 > msg.data.size()) {
                // copy的内容长度超过msg.data的长度
                msg.data.resize(frame_size + nalLength + 4);
                msg_temp = msg.data.data() + frame_size;
              }
              memcpy(msg_temp, data, nalLength + 4);
              msg_temp = msg_temp + nalLength + 4;
            }
            data = data + 4 + nalLength;  // 处理data中下一个NALU数据
          }

          msg.height = video_info_.height;
          msg.width = video_info_.width;
          memcpy(msg.encoding.data(), "h264", strlen("h264"));
          msg.index = ++video_info_.count_;
          if (video_info_.count_ == UINT_MAX) {
            video_info_.count_ = 0;
          }
          msg.data.resize(msg_temp - msg.data.data());
          struct timespec time_start = {0, 0};
          clock_gettime(CLOCK_REALTIME, &time_start);
          msg.dts.sec = time_start.tv_sec;
          msg.dts.nanosec = time_start.tv_nsec;
          msg.pts.sec = time_start.tv_sec;
          msg.pts.nanosec = time_start.tv_nsec;
          std::stringstream ss;
          ss << "publish ros video msg, file: " << video_info_.video_file_
             << ", topic: " << msg_pub_topic_name_
             << ", encoding: " << msg.encoding.data()
             << ", stamp: " << msg.dts.sec << "." << msg.dts.nanosec
             << ", dLen: " << msg.data.size();
          RCLCPP_INFO(
              rclcpp::get_logger("image_pub_node"), "%s", ss.str().c_str());
          ros_publisher_h26x_->publish(msg);
          std::this_thread::sleep_for(
              std::chrono::milliseconds((int)(1000.0 / video_info_.fps)));
        }
      }
      av_packet_unref(&packet);
    }
    avformat_close_input(&(video_info_.fmt_ctx));
    set_pub_index();
  } while (is_loop_ && rclcpp::ok());
}
