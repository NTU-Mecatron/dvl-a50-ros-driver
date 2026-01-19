#ifndef DVL_A50_PUBLISHER_HPP
#define DVL_A50_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

using namespace std;

class DVLA50Publisher : public rclcpp::Node
{
public:
    DVLA50Publisher();
    ~DVLA50Publisher();

private:
    void timer_callback();
    void connect();
    string getData();
    bool send_dvl_command(string cmd);
    void reset_dead_reckoning(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void calibrate_gyro(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void get_config(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void toggle(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_raw_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_dead_reckoning_server_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_gyro_server_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_config_server_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_server_;

    string reset_dead_reckoning_service, calibrate_gyro_service, get_config_service, toggle_service;

    int sock_;
    string tcp_ip_;
    int tcp_port_;
    string old_json_;
    string dvl_raw_topic;
};

#endif
