#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <dvl_a50_ros_driver/publisher.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto publisher = std::make_shared<DVLA50Publisher>(rclcpp::NodeOptions());

  try
  {
    rclcpp::spin(publisher);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(publisher->get_logger(), "Exception: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
