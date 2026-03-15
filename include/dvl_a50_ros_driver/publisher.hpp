#ifndef DVL_A50_PUBLISHER_HPP
#define DVL_A50_PUBLISHER_HPP

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

using String = std_msgs::msg::String;
using Trigger = std_srvs::srv::Trigger;
using SetBool = std_srvs::srv::SetBool;

class DVLA50Publisher : public rclcpp::Node
{
public:
  explicit DVLA50Publisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DVLA50Publisher();

private:
  void timer_callback();
  void connect();
  std::string getData();
  bool send_dvl_command(std::string cmd);
  void reset_dead_reckoning(const std::shared_ptr<Trigger::Request> req,
                            std::shared_ptr<Trigger::Response> res);
  void calibrate_gyro(const std::shared_ptr<Trigger::Request> req,
                      std::shared_ptr<Trigger::Response> res);
  void get_config(const std::shared_ptr<Trigger::Request> req,
                  std::shared_ptr<Trigger::Response> res);
  void toggle(const std::shared_ptr<SetBool::Request> req, std::shared_ptr<SetBool::Response> res);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<String>::SharedPtr pub_raw_;
  rclcpp::Service<Trigger>::SharedPtr reset_dead_reckoning_server_;
  rclcpp::Service<Trigger>::SharedPtr calibrate_gyro_server_;
  rclcpp::Service<Trigger>::SharedPtr get_config_server_;
  rclcpp::Service<SetBool>::SharedPtr toggle_server_;

  int sock_;
  std::string tcp_ip_;
  int tcp_port_;
  std::string old_json_;
};

#endif
