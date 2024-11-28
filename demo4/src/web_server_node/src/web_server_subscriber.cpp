#include "web_server_subscriber.hpp"

WebServerSubscriber::WebServerSubscriber() : Node("web_server_subscriber") {
    subscription_ = this->create_subscription<CustomImageMessage>(
        "camera_topic", 10,
        std::bind(&WebServerSubscriber::imageCallback, this, std::placeholders::_1)
    );
    shm_manager_.initialize("/camera_shared_memory", IMAGE_SIZE);

    httplib::Server svr;
    svr.Get("/", [&](const httplib::Request &, httplib::Response &res) {
        auto frame_data = shm_manager_.read();
        res.set_content(frame_data, "image/jpeg");
    });
    svr.listen("0.0.0.0", 8000);
}

void WebServerSubscriber::imageCallback(const CustomImageMessage::SharedPtr msg) {
    // Handle metadata here if necessary
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WebServerSubscriber>());
    rclcpp::shutdown();
    return 0;
}