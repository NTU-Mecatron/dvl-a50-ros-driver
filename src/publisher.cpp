#include <iomanip>
#include <iostream>

#include <dvl_a50_ros_driver/publisher.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

using json = nlohmann::json;

namespace dvl
{

RawJsonPublisher::RawJsonPublisher(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("dvl_a50_publisher", options), sock_(-1)
{
  // Declare and get parameters
  this->declare_parameter<std::string>("tcp_ip", "192.168.194.95");
  this->declare_parameter<int>("tcp_port", 16171);
  this->declare_parameter<std::string>("dvl_raw_topic", "dvl/raw_data");
  this->declare_parameter<std::string>("reset_dead_reckoning", "dvl/reset_dead_reckoning");
  this->declare_parameter<std::string>("calibrate_gyro", "dvl/calibrate_gyro");
  this->declare_parameter<std::string>("get_config", "dvl/get_config");
}

CallbackReturn RawJsonPublisher::on_configure(const rclcpp_lifecycle::State &) {
  tcp_ip_ = this->get_parameter("tcp_ip").as_string();
  tcp_port_ = this->get_parameter("tcp_port").as_int();

  const std::string dvl_raw_topic = this->get_parameter("dvl_raw_topic").as_string();
  const std::string reset_dead_reckoning_service =
      this->get_parameter("reset_dead_reckoning").as_string();
  const std::string calibrate_gyro_service = this->get_parameter("calibrate_gyro").as_string();
  const std::string get_config_service = this->get_parameter("get_config").as_string();

  // Create publisher for raw JSON data
  pub_raw_ = this->create_publisher<String>(dvl_raw_topic, 10);

  // Create services
  reset_dead_reckoning_server_ = this->create_service<Trigger>(
      reset_dead_reckoning_service, std::bind(&RawJsonPublisher::reset_dead_reckoning, this,
                                              std::placeholders::_1, std::placeholders::_2));
  calibrate_gyro_server_ = this->create_service<Trigger>(
      calibrate_gyro_service, std::bind(&RawJsonPublisher::calibrate_gyro, this,
                                        std::placeholders::_1, std::placeholders::_2));
  get_config_server_ = this->create_service<Trigger>(
      get_config_service,
      std::bind(&RawJsonPublisher::get_config, this, std::placeholders::_1, std::placeholders::_2));

  // Set up the socket connection
  RCLCPP_INFO(this->get_logger(), "Connecting to DVL at %s:%d", tcp_ip_.c_str(), tcp_port_);
  RCLCPP_INFO(this->get_logger(), "Configure transition is called.");
  connect_socket();

  // Turning off upon Inactive
  bool success = send_dvl_command("\"set_config\",\"parameters\":{\"acoustic_enabled\":false}");
  const char* message = success ? "DVL turned off" : "Failed to toggle DVL";
  RCLCPP_INFO(this->get_logger(), message);

  if (!success) { return CallbackReturn::FAILURE; }

  // Reset dead reckoning on startup
  auto starting_req = std::make_shared<Trigger::Request>();
  auto starting_res = std::make_shared<Trigger::Response>();
  reset_dead_reckoning(starting_req, starting_res);

  return CallbackReturn::SUCCESS;
}

CallbackReturn RawJsonPublisher::on_activate(const rclcpp_lifecycle::State &) {
  pub_raw_->on_activate();

  // Turning on during ACTIVE only
  bool success = send_dvl_command("\"set_config\",\"parameters\":{\"acoustic_enabled\":true}");
  const char* message = success ? "DVL turned on" : "Failed to toggle DVL";
  RCLCPP_INFO(this->get_logger(), message);

  if (!success) { return CallbackReturn::FAILURE; }

  // Create timer for periodic data collection (30 Hz)
  timer_ = this->create_timer(std::chrono::milliseconds(33),  // ~30 Hz
                                   std::bind(&RawJsonPublisher::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Activate transition is called.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RawJsonPublisher::on_deactivate(const rclcpp_lifecycle::State &) {
  // Turning off
  bool success = send_dvl_command("\"set_config\",\"parameters\":{\"acoustic_enabled\":false}");
  const char* message = success ? "DVL turned off" : "Failed to toggle DVL";
  RCLCPP_INFO(this->get_logger(), message);

  if (!success) { return CallbackReturn::FAILURE; }

  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
  
  pub_raw_->on_deactivate();

  RCLCPP_INFO(this->get_logger(), "Deactivate transition is called.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RawJsonPublisher::on_cleanup(const rclcpp_lifecycle::State &) {
  // Turning off
  bool success = send_dvl_command("\"set_config\",\"parameters\":{\"acoustic_enabled\":false}");
  const char* message = success ? "DVL turned off" : "Failed to toggle DVL";
  RCLCPP_INFO(this->get_logger(), message);

  if (!success) { return CallbackReturn::FAILURE; }

  close_socket();

  if (timer_) {
    timer_->cancel();
  }
  timer_.reset();
  pub_raw_.reset();

  reset_dead_reckoning_server_.reset();
  calibrate_gyro_server_.reset();
  get_config_server_.reset();

  RCLCPP_INFO(this->get_logger(), "Cleanup transition is called.");
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn RawJsonPublisher::on_shutdown(const rclcpp_lifecycle::State &) {
  // Turning off
  bool success = send_dvl_command("\"set_config\",\"parameters\":{\"acoustic_enabled\":false}");
  const char* message = success ? "DVL turned off" : "Failed to toggle DVL";
  RCLCPP_INFO(this->get_logger(), message);

  if (!success) { return CallbackReturn::FAILURE; }

  if (timer_) {
    timer_->cancel();
  }
  timer_.reset();
  pub_raw_.reset();

  reset_dead_reckoning_server_.reset();
  calibrate_gyro_server_.reset();
  get_config_server_.reset();

  close_socket();

  RCLCPP_INFO(this->get_logger(), "Shutdown transition is called.");
  return CallbackReturn::SUCCESS;
}


RawJsonPublisher::~RawJsonPublisher()
{
  if (sock_ >= 0)
  {
    close(sock_);
  }
}

void RawJsonPublisher::connect_socket()
{
  if (sock_ >= 0)
  {
    close(sock_);
  }

  sock_ = socket(AF_INET, SOCK_STREAM, 0);
  if (sock_ < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Socket creation error");
    rclcpp::sleep_for(std::chrono::seconds(1));
    connect_socket();
    return;
  }

  struct sockaddr_in serv_addr;
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(tcp_port_);

  if (inet_pton(AF_INET, tcp_ip_.c_str(), &serv_addr.sin_addr) <= 0)
  {
    RCLCPP_WARN(this->get_logger(), "Invalid address");
    rclcpp::sleep_for(std::chrono::seconds(1));
    connect_socket();
    return;
  }

  if (::connect(sock_, (struct sockaddr*) &serv_addr, sizeof(serv_addr)) < 0)
  {
    RCLCPP_WARN(this->get_logger(), "Connection failed");
    rclcpp::sleep_for(std::chrono::seconds(1));
    connect_socket();
    return;
  }

  struct timeval tv;
  tv.tv_sec = 1;
  tv.tv_usec = 0;
  setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, (const char*) &tv, sizeof tv);
}

void RawJsonPublisher::close_socket() {
  if (sock_ >= 0) { close(sock_); sock_ = -1; }
}

std::string RawJsonPublisher::getData()
{
  std::string raw_data;
  std::vector<char> buffer(1024);

  while (raw_data.find('\n') == std::string::npos)
  {
    ssize_t n = recv(sock_, buffer.data(), buffer.size(), 0);
    if (n < 1)
    {
      RCLCPP_WARN(this->get_logger(), "Connection lost, reconnecting...");
      connect_socket();
      continue;
    }
    raw_data.append(buffer.data(), n);
  }

  raw_data = old_json_ + raw_data;
  old_json_ = "";

  size_t pos = raw_data.find('\n');

  // check if we received the full json string
  if (pos != std::string::npos)
  {
    old_json_ = raw_data.substr(pos + 1);
    raw_data = raw_data.substr(0, pos);
  }

  return raw_data;
}

bool RawJsonPublisher::send_dvl_command(std::string cmd)
{
  std::string full_cmd{"{\"command\": " + cmd + "}"};
  RCLCPP_WARN(this->get_logger(), "sending %s", full_cmd.c_str());
  send(sock_, full_cmd.c_str(), full_cmd.size(), 0);

  const int max_retries = 5;
  int retry_count = 0;

  while (retry_count < max_retries)
  {
    std::string _dr_status_response = getData();
    try
    {
      json resp = json::parse(_dr_status_response);
      std::string response_to = resp["response_to"];
      if (resp["type"] == "response" && (cmd.find(response_to) != std::string::npos))
      {
        if (resp["success"])
        {
          RCLCPP_WARN(this->get_logger(), "%s successful", cmd.c_str());
          auto result{resp["result"]};
          if (result == NULL)
          {
            RCLCPP_WARN(this->get_logger(), "No result, likely expected null type return");
          }
          else
          {
            try
            {
              RCLCPP_WARN(this->get_logger(), "DVL Result:\n%s", result.dump(2).c_str());
            }
            catch (const std::exception& e)
            {
              RCLCPP_WARN(this->get_logger(), "Error in returning result: %s", e.what());
            }
          }
          rclcpp::sleep_for(std::chrono::milliseconds(50));  // wait 50ms for values to zero out
          return true;
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Dead reckoning reset failed: %s",
                       resp["error_message"].get<std::string>().c_str());
          return false;
        }
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Unexpected response to command: %s",
                    _dr_status_response.c_str());
        return false;
      }
    }
    catch (const std::exception& e)
    {
      retry_count++;
      if (retry_count >= max_retries)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse response after %d attempts: %s",
                     retry_count, e.what());
        return false;
      }
      RCLCPP_ERROR(this->get_logger(), "Failed to parse reset response (%d/%d): %s", retry_count,
                   max_retries, e.what());
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      continue;
    }
  }

  return false;
}

void RawJsonPublisher::reset_dead_reckoning(const std::shared_ptr<Trigger::Request> req,
                                           std::shared_ptr<Trigger::Response> res)
{
  (void) req;
  bool success = send_dvl_command("\"reset_dead_reckoning\"");
  res->success = success;
  res->message = success ? "Dead reckoning reset successful" : "Dead reckoning reset failed";
}

void RawJsonPublisher::calibrate_gyro(const std::shared_ptr<Trigger::Request> req,
                                     std::shared_ptr<Trigger::Response> res)
{
  (void) req;
  RCLCPP_WARN(this->get_logger(),
              "Temporary disconnection when calibrating gyro, expect NULL return");
  bool success = send_dvl_command("\"calibrate_gyro\"");
  res->success = success;
  res->message = success ? "Calibrate gyro successful" : "Calibrate gyro failed";
}

void RawJsonPublisher::get_config(const std::shared_ptr<Trigger::Request> req,
                                 std::shared_ptr<Trigger::Response> res)
{
  (void) req;
  bool success = send_dvl_command("\"get_config\"");
  res->success = success;
  res->message = success ? "Get config successful" : "Get config failed";
}

void RawJsonPublisher::timer_callback()
{
  std::string raw_data = getData();

  // Publish raw JSON data for downstream processing
  String raw_msg;
  raw_msg.data = raw_data;
  pub_raw_->publish(raw_msg);
}

}  // namespace dvl

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dvl::RawJsonPublisher)
