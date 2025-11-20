#include <dvl_a50_ros_driver/subscriber.hpp>

DVLA50Subscriber::DVLA50Subscriber() : Node("dvl_a50_subscriber") {
    raw_sub_ = this->create_subscription<std_msgs::msg::String>(
        "dvl/json_data", 10,
        std::bind(&DVLA50Subscriber::callbackRAW, this, std::placeholders::_1));
    dvl_sub_ = this->create_subscription<dvl_a50_ros_driver::msg::DVL>(
        "dvl/data", 10,
        std::bind(&DVLA50Subscriber::callback, this, std::placeholders::_1));
}

void DVLA50Subscriber::callbackRAW(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Data received: %s", msg->data.c_str());
}

void DVLA50Subscriber::callback(const dvl_a50_ros_driver::msg::DVL::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Time received: %f", msg->time);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto subscriber = std::make_shared<DVLA50Subscriber>();
    rclcpp::spin(subscriber);
    rclcpp::shutdown();
    return 0;
}
