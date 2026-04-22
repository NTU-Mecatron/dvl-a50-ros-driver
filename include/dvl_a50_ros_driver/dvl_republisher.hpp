#ifndef DVL_A50_REPUBLISHER_HPP
#define DVL_A50_REPUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include <nlohmann/json.hpp>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>

namespace dvl
{

using json = nlohmann::json;
using String = std_msgs::msg::String;
using TwistWithCovarianceStamped = geometry_msgs::msg::TwistWithCovarianceStamped;

class TwistPublisher : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit TwistPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  CallbackReturn on_configure (const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate (const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate (const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown (const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup (const rclcpp_lifecycle::State &) override;

private:
  void raw_dvl_callback(const String::ConstSharedPtr msg);

  // Initialization helpers
  void load_parameters();
  // void setup_pub_sub();
  void setup_pub();
  void setup_sub();
  void initialize_twist_template();

  // Message processing helpers
  bool is_velocity_valid(const json& data);
  void populate_linear_velocities(TwistWithCovarianceStamped& twist_msg, const json& data) const;
  void compute_velocity_covariances(double& variance_x, double& variance_y, double& variance_z,
                                    const json& data) const;
  rclcpp::Subscription<String>::SharedPtr dvl_raw_sub_;
  rclcpp_lifecycle::LifecyclePublisher<TwistWithCovarianceStamped>::SharedPtr dvl_twist_pub_;

  std::string dvl_frame_id_;
  TwistWithCovarianceStamped twist_template_;
  bool use_original_covariance_;
  bool use_fom_to_compute_covariance_;
  double covariance_multiplier_;
  double linear_vel_var_x_;
  double linear_vel_var_y_;
  double linear_vel_var_z_;
};

}  // namespace dvl

#endif
