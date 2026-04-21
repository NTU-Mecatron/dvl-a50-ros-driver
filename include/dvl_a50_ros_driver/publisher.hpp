#ifndef DVL_A50_PUBLISHER_HPP
#define DVL_A50_PUBLISHER_HPP

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>

namespace dvl
{

using String = std_msgs::msg::String;
using Trigger = std_srvs::srv::Trigger;
using SetBool = std_srvs::srv::SetBool;

class RawJsonPublisher : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit RawJsonPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RawJsonPublisher();

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  CallbackReturn on_configure (const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate (const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate (const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown (const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup (const rclcpp_lifecycle::State &) override;

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
  rclcpp_lifecycle::LifecyclePublisher<String>::SharedPtr pub_raw_;
  rclcpp::Service<Trigger>::SharedPtr reset_dead_reckoning_server_;
  rclcpp::Service<Trigger>::SharedPtr calibrate_gyro_server_;
  rclcpp::Service<Trigger>::SharedPtr get_config_server_;
  rclcpp::Service<SetBool>::SharedPtr toggle_server_;

  int sock_;
  std::string tcp_ip_;
  int tcp_port_;
  std::string old_json_;
};

}  // namespace dvl

#endif

