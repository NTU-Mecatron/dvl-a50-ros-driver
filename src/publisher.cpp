#include <dvl_a50_ros_driver/publisher.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
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

    string dvl_topic, dvl_raw_topic, dead_reckoning_topic;
    nh_.param<string>("dvl_topic", dvl_topic, "dvl/velocity");
    nh_.param<string>("dvl_raw_topic", dvl_raw_topic, "dvl/raw_data");
    nh_.param<string>("dead_reckoning_topic", dead_reckoning_topic, "dvl/dead_reckoning");

    pub_raw_ = nh_.advertise<String>(dvl_raw_topic, 10);
    pub_velocity_ = nh_.advertise<DVL>(dvl_topic, 10);
    pub_dead_reckoning_ = nh_.advertise<DVLDeadReckoning>(dead_reckoning_topic, 10);

    // Set up the socket connection
    ROS_INFO("Connecting to DVL at %s:%d", tcp_ip_.c_str(), tcp_port_);
    connect();

    string reset_dead_reckoning = "{\"command\": \"reset_dead_reckoning\"}";
    send(sock_, reset_dead_reckoning.c_str(), reset_dead_reckoning.size(), 0);
    ROS_INFO("Reset Dead Reckoning command sent");

    ros::Duration(0.5).sleep(); // wait before polling dr status success
    string _dr_status_response = getData();
    try {
        json resp = json::parse(_dr_status_response);
        if (resp["type"] == "response" && resp["response_to"] == "reset_dead_reckoning") {
            if (resp["success"]) {
                ROS_INFO("Dead reckoning reset successful");
                ros::Duration(0.05).sleep(); // wait 50ms for values to zero out
            } else {
                ROS_ERROR("Dead reckoning reset failed: %s", 
                         resp["error_message"].get<string>().c_str());
            }
        } else {
            ROS_WARN("Unexpected response to reset command: %s", _dr_status_response.c_str());
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to parse reset response: %s", e.what());
    }

    if (do_log_raw_data_)
        ROS_INFO("Logging raw data to topic: %s", dvl_raw_topic.c_str());
    else
        ROS_INFO("Publishing DVL data to two topics: %s and %s", dvl_topic.c_str(), dead_reckoning_topic.c_str());
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
