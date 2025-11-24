#ifndef DVL_A50_SUBSCRIBER_HPP
#define DVL_A50_SUBSCRIBER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <dvl_a50_ros_driver/msg/dvl.hpp>
#include <dvl_a50_ros_driver/msg/dvl_beam.hpp>

class DVLA50Subscriber : public rclcpp::Node
{
public:
    DVLA50Subscriber();

private:
    void callbackRAW(const std_msgs::msg::String::SharedPtr msg);
    void callback(const dvl_a50_ros_driver::msg::DVL::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr raw_sub_;
    rclcpp::Subscription<dvl_a50_ros_driver::msg::DVL>::SharedPtr dvl_sub_;
};

#endif
