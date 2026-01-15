#include <dvl_a50_ros_driver/publisher.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <iomanip>

using json = nlohmann::json;
using String = std_msgs::msg::String;

DVLA50Publisher::DVLA50Publisher() : Node("dvl_a50_publisher"), sock_(-1)
{
    // Declare and get parameters
    this->declare_parameter<string>("tcp_ip", "192.168.194.95");
    this->declare_parameter<int>("tcp_port", 16171);
    this->declare_parameter<string>("dvl_raw_topic", "dvl/raw_data");
    this->declare_parameter<string>("reset_dead_reckoning", "dvl/reset_dead_reckoning");
    this->declare_parameter<string>("calibrate_gyro", "dvl/calibrate_gyro");
    this->declare_parameter<string>("get_config", "dvl/get_config");
    this->declare_parameter<string>("turn_off", "dvl/turn_off");
    this->declare_parameter<string>("turn_on", "dvl/turn_on");
    this->declare_parameter<string>("toggle", "dvl/toggle");

    tcp_ip_ = this->get_parameter("tcp_ip").as_string();
    tcp_port_ = this->get_parameter("tcp_port").as_int();
    dvl_raw_topic = this->get_parameter("dvl_raw_topic").as_string();
    reset_dead_reckoning_service = this->get_parameter("reset_dead_reckoning").as_string();
    calibrate_gyro_service = this->get_parameter("calibrate_gyro").as_string();
    get_config_service = this->get_parameter("get_config").as_string();
    turn_off_service = this->get_parameter("turn_off").as_string();
    turn_on_service = this->get_parameter("turn_on").as_string();
    toggle_service = this->get_parameter("toggle").as_string();

    // Create publisher for raw JSON data
    pub_raw_ = this->create_publisher<String>(dvl_raw_topic, 10);

    // Create services
    reset_dead_reckoning_server_ = this->create_service<std_srvs::srv::Trigger>(
        reset_dead_reckoning_service,
        std::bind(&DVLA50Publisher::reset_dead_reckoning, this, std::placeholders::_1, std::placeholders::_2));
    calibrate_gyro_server_ = this->create_service<std_srvs::srv::Trigger>(
        calibrate_gyro_service,
        std::bind(&DVLA50Publisher::calibrate_gyro, this, std::placeholders::_1, std::placeholders::_2));
    get_config_server_ = this->create_service<std_srvs::srv::Trigger>(
        get_config_service,
        std::bind(&DVLA50Publisher::get_config, this, std::placeholders::_1, std::placeholders::_2));
    turn_off_server_ = this->create_service<std_srvs::srv::Trigger>(
        turn_off_service,
        std::bind(&DVLA50Publisher::turn_off, this, std::placeholders::_1, std::placeholders::_2));
    turn_on_server_ = this->create_service<std_srvs::srv::Trigger>(
        turn_on_service,
        std::bind(&DVLA50Publisher::turn_on, this, std::placeholders::_1, std::placeholders::_2));
    toggle_server_ = this->create_service<std_srvs::srv::SetBool>(
        toggle_service,
        std::bind(&DVLA50Publisher::toggle, this, std::placeholders::_1, std::placeholders::_2));

    // Set up the socket connection
    RCLCPP_INFO(this->get_logger(), "Connecting to DVL at %s:%d", tcp_ip_.c_str(), tcp_port_);
    connect();

    // Reset dead reckoning on startup
    auto starting_req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto starting_res = std::make_shared<std_srvs::srv::Trigger::Response>();
    reset_dead_reckoning(starting_req, starting_res);

    // Create timer for periodic data collection (30 Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33), // ~30 Hz
        std::bind(&DVLA50Publisher::timer_callback, this));
}

DVLA50Publisher::~DVLA50Publisher()
{
    if (sock_ >= 0)
    {
        close(sock_);
    }
}

void DVLA50Publisher::connect()
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
        connect();
        return;
    }

    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(tcp_port_);

    if (inet_pton(AF_INET, tcp_ip_.c_str(), &serv_addr.sin_addr) <= 0)
    {
        RCLCPP_WARN(this->get_logger(), "Invalid address");
        rclcpp::sleep_for(std::chrono::seconds(1));
        connect();
        return;
    }

    if (::connect(sock_, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        RCLCPP_WARN(this->get_logger(), "Connection failed");
        rclcpp::sleep_for(std::chrono::seconds(1));
        connect();
        return;
    }

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);
}

string DVLA50Publisher::getData()
{
    string raw_data;
    vector<char> buffer(1024);

    while (raw_data.find('\n') == string::npos)
    {
        ssize_t n = recv(sock_, buffer.data(), buffer.size(), 0);
        if (n < 1)
        {
            RCLCPP_WARN(this->get_logger(), "Connection lost, reconnecting...");
            connect();
            continue;
        }
        raw_data.append(buffer.data(), n);
    }

    raw_data = old_json_ + raw_data;
    old_json_ = "";

    size_t pos = raw_data.find('\n');

    // check if we received the full json string
    if (pos != string::npos)
    {
        old_json_ = raw_data.substr(pos + 1);
        raw_data = raw_data.substr(0, pos);
    }

    return raw_data;
}

bool DVLA50Publisher::send_dvl_command(string cmd)
{
    string full_cmd{"{\"command\": " + cmd + "}"};
    RCLCPP_WARN(this->get_logger(), "sending %s", full_cmd.c_str());
    send(sock_, full_cmd.c_str(), full_cmd.size(), 0);

    const int max_retries = 5;
    int retry_count = 0;

    while (retry_count < max_retries)
    {
        string _dr_status_response = getData();
        try
        {
            json resp = json::parse(_dr_status_response);
            string response_to = resp["response_to"];
            if (resp["type"] == "response" && (cmd.find(response_to) != string::npos))
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
                        catch (const std::exception &e)
                        {
                            RCLCPP_WARN(this->get_logger(), "Error in returning result: %s", e.what());
                        }
                    }
                    rclcpp::sleep_for(std::chrono::milliseconds(50)); // wait 50ms for values to zero out
                    return true;
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Dead reckoning reset failed: %s",
                                 resp["error_message"].get<string>().c_str());
                    return false;
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Unexpected response to command: %s", _dr_status_response.c_str());
                return false;
            }
        }
        catch (const std::exception &e)
        {
            retry_count++;
            if (retry_count >= max_retries)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse response after %d attempts: %s", retry_count, e.what());
                return false;
            }
            RCLCPP_ERROR(this->get_logger(), "Failed to parse reset response (%d/%d): %s", retry_count, max_retries, e.what());
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
    }

    return false;
}

void DVLA50Publisher::reset_dead_reckoning(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    (void)req;
    bool success = send_dvl_command("\"reset_dead_reckoning\"");
    res->success = success;
    res->message = success ? "Dead reckoning reset successful" : "Dead reckoning reset failed";
}

void DVLA50Publisher::calibrate_gyro(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    (void)req;
    RCLCPP_WARN(this->get_logger(), "Temporary disconnection when calibrating gyro, expect NULL return");
    bool success = send_dvl_command("\"calibrate_gyro\"");
    res->success = success;
    res->message = success ? "Calibrate gyro successful" : "Calibrate gyro failed";
}

void DVLA50Publisher::toggle(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
    bool success = send_dvl_command(req->data ? "\"set_config\",\"parameters\":{\"acoustic_enabled\":true}" : "\"set_config\",\"parameters\":{\"acoustic_enabled\":false}");
    res->success = success;
    res->message = success ? (req->data ? "DVL turned on" : "DVL turned off") : "Failed to toggle DVL";
}

void DVLA50Publisher::turn_off(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    (void)req;
    bool success = send_dvl_command("\"set_config\",\"parameters\":{\"acoustic_enabled\":false}");
    res->success = success;
    res->message = success ? "turn_off successful" : "turn_off failed";
}

void DVLA50Publisher::turn_on(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    (void)req;
    bool success = send_dvl_command("\"set_config\",\"parameters\":{\"acoustic_enabled\":true}");
    res->success = success;
    res->message = success ? "turn_on successful" : "turn_on failed";
}

void DVLA50Publisher::get_config(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    (void)req;
    bool success = send_dvl_command("\"get_config\"");
    res->success = success;
    res->message = success ? "Get config successful" : "Get config failed";
}

void DVLA50Publisher::timer_callback()
{
    string raw_data = getData();

    // Publish raw JSON data for downstream processing
    String raw_msg;
    raw_msg.data = raw_data;
    pub_raw_->publish(raw_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto publisher = std::make_shared<DVLA50Publisher>();

    try
    {
        rclcpp::spin(publisher);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(publisher->get_logger(), "Exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
