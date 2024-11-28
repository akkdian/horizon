#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <curl/curl.h>
#include <iostream>
#include <fstream>
#include <vector>

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber() : Node("image_subscriber")
    {
        // 创建图像订阅者
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10,
            std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1));

        curl_global_init(CURL_GLOBAL_DEFAULT);
    }

    ~ImageSubscriber()
    {
        curl_global_cleanup();
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // 发送图像数据到 HTTP 服务器
        send_image_to_server(msg);
    }

    void send_image_to_server(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        CURL *curl = curl_easy_init();
        if (curl)
        {
            // HTTP 服务器的地址
            const std::string server_url = "http://localhost:8000/upload_image";

            // 设置 CURL 请求头
            struct curl_slist *headers = NULL;
            headers = curl_slist_append(headers, "Content-Type: application/octet-stream");

            // 发送图像数据
            curl_easy_setopt(curl, CURLOPT_URL, server_url.c_str());
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, msg->data.data());
            curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, msg->data.size());

            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to send image: %s", curl_easy_strerror(res));
            }

            // 清理
            curl_easy_cleanup(curl);
            curl_slist_free_all(headers);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}
