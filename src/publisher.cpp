#include <dvl_a50_ros_driver/publisher.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <iomanip>
#include <dvl_a50_ros_driver/DVL.h>
#include <dvl_a50_ros_driver/DVLBeam.h>
#include <dvl_a50_ros_driver/DVLDeadReckoning.h>

using json = nlohmann::json;
using DVL = dvl_a50_ros_driver::DVL;
using DVLBeam = dvl_a50_ros_driver::DVLBeam;
using DVLDeadReckoning = dvl_a50_ros_driver::DVLDeadReckoning;
using String = std_msgs::String;

DVLA50Publisher::DVLA50Publisher(ros::NodeHandle& nh) : nh_(nh), sock_(-1) 
{
    // Initialize the socket
    nh_.param<string>("tcp_ip", tcp_ip_, "192.168.194.95");
    nh_.param<int>("tcp_port", tcp_port_, 16171);
    nh_.param<bool>("log_raw_data", do_log_raw_data_, false);

    nh_.param<string>("dvl_topic", dvl_topic, "dvl/velocity");
    nh_.param<string>("dvl_raw_topic", dvl_raw_topic, "dvl/raw_data");
    nh_.param<string>("dead_reckoning_topic", dead_reckoning_topic, "dvl/dead_reckoning");

    pub_raw_ = nh_.advertise<String>(dvl_raw_topic, 10);
    pub_velocity_ = nh_.advertise<DVL>(dvl_topic, 10);
    pub_dead_reckoning_ = nh_.advertise<DVLDeadReckoning>(dead_reckoning_topic, 10);

    nh_.param<string>("reset_dead_reckoning", reset_dead_reckoning_service, "dvl/reset_dead_reckoning");
    nh_.param<string>("calibrate_gyro", calibrate_gyro_service, "dvl/calibrate_gyro");
    nh_.param<string>("get_config", get_config_service, "dvl/get_config");
    nh_.param<string>("turn_off", turn_off_service, "dvl/turn_off");
    nh_.param<string>("turn_on", turn_on_service, "dvl/turn_on");
    nh_.param<string>("toggle", turn_on_service, "dvl/toggle");
    reset_dead_reckoning_server_ = nh_.advertiseService(reset_dead_reckoning_service, &DVLA50Publisher::reset_dead_reckoning, this);
    calibrate_gyro_server_ = nh_.advertiseService(calibrate_gyro_service, &DVLA50Publisher::calibrate_gyro, this);
    get_config_server_ = nh_.advertiseService(get_config_service, &DVLA50Publisher::get_config, this);
    turn_off_server_  = nh_.advertiseService(turn_off_service, &DVLA50Publisher::turn_off, this);
    turn_on_server_ = nh_.advertiseService(turn_on_service, &DVLA50Publisher::turn_on, this);
    toggle_server = nh_.advertiseService(toggle_service, &DVLA50Publisher::toggle, this);

    // Set up the socket connection
    ROS_INFO("Connecting to DVL at %s:%d", tcp_ip_.c_str(), tcp_port_);
    connect();

    std_srvs::Trigger::Request starting_req;
    std_srvs::Trigger::Response starting_res;
    reset_dead_reckoning(starting_req, starting_res);
}


DVLA50Publisher::~DVLA50Publisher() 
{
    if (sock_ >= 0) {
        close(sock_);
    }
}

void DVLA50Publisher::connect() 
{
    if (sock_ >= 0) {
        close(sock_);
    }

    sock_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_ < 0) {
        ROS_ERROR("Socket creation error");
        ros::Duration(1.0).sleep();
        connect();
        return;
    }

    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(tcp_port_);

    if (inet_pton(AF_INET, tcp_ip_.c_str(), &serv_addr.sin_addr) <= 0) {
        ROS_WARN("Invalid address");
        ros::Duration(1.0).sleep();
        connect();
        return;
    }

    if (::connect(sock_, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        ROS_WARN("Connection failed");
        ros::Duration(1.0).sleep();
        connect();
        return;
    }

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
}

string DVLA50Publisher::getData() 
{
    string raw_data;
    vector<char> buffer(1024);
    
    while (raw_data.find('\n') == string::npos) {
        ssize_t n = recv(sock_, buffer.data(), buffer.size(), 0);
        if (n < 1) {
            ROS_WARN("Connection lost, reconnecting...");
            connect();
            continue;
        }
        raw_data.append(buffer.data(), n);
    }

    raw_data = old_json_ + raw_data;
    old_json_ = "";

    size_t pos = raw_data.find('\n');

    // check if we received the full json string
    if (pos != string::npos) {
        old_json_ = raw_data.substr(pos + 1);
        raw_data = raw_data.substr(0, pos);
    }

    return raw_data;
}

bool DVLA50Publisher::send_dvl_command(string cmd) {
    string full_cmd {"{\"command\": " + cmd + "}"}; 
    ROS_WARN("sending %s", full_cmd.c_str());
    send(sock_, full_cmd.c_str(), full_cmd.size(), 0);

    const int max_retries = 5;
    int retry_count = 0;

    while (retry_count < max_retries) {
        string _dr_status_response = getData();
        try {
            json resp = json::parse(_dr_status_response);
            string response_to = resp["response_to"];
            if (resp["type"] == "response" && (cmd.find(response_to) != string::npos)) {
                if (resp["success"]) {
                    ROS_WARN("%s successful", cmd.c_str());
                    auto result {resp["result"]};
                    if (result == NULL) {
                        ROS_WARN("No result, likely expected null type return");
                    } else {
                        try {
                            ROS_WARN("DVL Result:\n%s", result.dump(2).c_str());
                        } catch (const std::exception& e) {
                            ROS_WARN("Error in returning result: %s", e.what());
                        }
                    }
                    ros::Duration(0.05).sleep(); // wait 50ms for values to zero out
                    return true;
                } else {
                    ROS_ERROR("Dead reckoning reset failed: %s", 
                            resp["error_message"].get<string>().c_str());
                    return false;
                }
            } else {
                ROS_WARN("Unexpected response to command: %s", _dr_status_response.c_str());
                return false;
            }
        } catch (const std::exception& e) {
            retry_count++;
            if (retry_count >= max_retries) {
                ROS_ERROR("Failed to parse response after %d attempts: %s", retry_count, e.what());
                return false;
            }
            ROS_ERROR("Failed to parse reset response (%d/%d): %s", retry_count, max_retries, e.what());
            ros::Duration(0.1).sleep();
            continue;
        }
    }

    if (do_log_raw_data_) {
        ROS_INFO("Logging raw data to topic: %s", dvl_raw_topic.c_str());
        return true;
    }
    else {
        ROS_INFO("Publishing DVL data to two topics: %s and %s", dvl_topic.c_str(), dead_reckoning_topic.c_str());
        return true;
    }
}

bool DVLA50Publisher::reset_dead_reckoning(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    bool success = send_dvl_command("\"reset_dead_reckoning\"");
    res.success = success;
    res.message = success ? "Dead reckoning reset successful" : "Dead reckoning reset failed";
    return true;
}

bool DVLA50Publisher::calibrate_gyro(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    ROS_WARN("Temporary disconnection when calibrating gyro, expect NULL return")
    bool success = send_dvl_command("\"calibrate_gyro\"");
    res.success = success;
    res.message = success ? "Calibrate gyro successful" : "Calibrate gyro failed";
    return true;
}

bool DVLA50Publisher::toggle(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    bool success = send_dvl_command(req.data ? "\"set_config\",\"parameters\":{\"acoustic_enabled\":true}" : "\"set_config\",\"parameters\":{\"acoustic_enabled\":false}");
    res.success = success;
    res.message = success ? (req.data ? "DVL turned on" : "DVL turned off") : "Failed to toggle DVL";
    return true;
}

bool DVLA50Publisher::turn_off(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    bool success = send_dvl_command("\"set_config\",\"parameters\":{\"acoustic_enabled\":false}");
    res.success = success;
    res.message = success ? "turn_off successful" : "turn_off failed";
    return true;
}

bool DVLA50Publisher::turn_on(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    bool success = send_dvl_command("\"set_config\",\"parameters\":{\"acoustic_enabled\":true}");
    res.success = success;
    res.message = success ? "turn_on successful" : "turn_on failed";
    return true;
}

bool DVLA50Publisher::get_config(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    bool success = send_dvl_command("\"get_config\"");
    res.success = success;
    res.message = success ? "Get config successful" : "Get config failed";
    return true;
}

void DVLA50Publisher::run() 
{
    ros::Rate loop_rate(30); // Hz
    while (ros::ok()) 
    {
        string raw_data = getData();
        json data = json::parse(raw_data);

        if (do_log_raw_data_) {
            String raw_msg;
            raw_msg.data = raw_data;
            ROS_INFO("%s", raw_data.c_str());
            pub_raw_.publish(raw_msg);
            continue;
        }

        // Handle both velocity and position messages
        if (data["type"] == "position_local") 
        {
            // Update position and attitude data
            DVLDeadReckoning dr_msg;
            dr_msg.attitude = {data["roll"], data["pitch"], data["yaw"]};
            dr_msg.position = {data["x"], data["y"], data["z"]};
            dr_msg.std = data["std"];
            dr_msg.ts = data["ts"];
            pub_dead_reckoning_.publish(dr_msg);
        }
        else if (data["type"] == "velocity") 
        {
            DVL dvl_msg;
            dvl_msg.header.stamp = ros::Time::now();
            dvl_msg.header.frame_id = "dvl_link";
            dvl_msg.time = data["time"];
            dvl_msg.velocity.x = data["vx"];
            dvl_msg.velocity.y = data["vy"];
            dvl_msg.velocity.z = data["vz"];
            dvl_msg.fom = data["fom"];
            dvl_msg.altitude = data["altitude"];
            dvl_msg.velocity_valid = data["velocity_valid"];
            dvl_msg.status = data["status"];
            dvl_msg.form = data["format"];

            for (int i = 0; i < 4; ++i) {
                DVLBeam beam;
                const auto& trans = data["transducers"][i];
                
                beam.id = trans["id"];
                beam.velocity = trans["velocity"];
                beam.distance = trans["distance"];
                beam.rssi = trans["rssi"];
                beam.nsd = trans["nsd"];
                beam.valid = trans["beam_valid"];
                dvl_msg.beams.push_back(beam);
            }
            pub_velocity_.publish(dvl_msg);
        }
        else {
            continue;
        }
	ros::spinOnce();

        loop_rate.sleep();
    }
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "a50_pub");
    ros::NodeHandle nh("~");
    DVLA50Publisher publisher(nh);
    
    try {
        publisher.run();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }

    return 0;
}
