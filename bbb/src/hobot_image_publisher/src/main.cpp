#include <memory>
#include <string>

#include "include/image_pub_node.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<PubNode>("hobot_image_pub"));

  rclcpp::shutdown();
  return 0;
}
