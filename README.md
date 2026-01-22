# Water Linked DVL A50 - ROS2 Driver

A ROS2 C++ package for the Water Linked DVL A50 Doppler Velocity Log. Provides TCP/IP communication with the DVL hardware and converts raw JSON data to standard ROS2 TwistWithCovarianceStamped messages.

Water Linked A50 is, by far, the world's smallest commercially available Doppler Velocity Log. With the record-breaking 5 cm min altitude measurability, the A50 is extremely useful for working with tools close to the seabed.

![Image of Water Linked A50](img/DSC04478_1600_web.jpg?raw=true "Water Linked DVL A50")

### Prerequisites
- **ROS2 Humble** on Ubuntu 22.04
- Dependencies:
  ```bash
  sudo apt-get install nlohmann-json3-dev
  ```

## Installation
Clone into your ROS2 workspace source directory and build:
```bash
cd ~/auv_ws/src
git clone git@github.com:NTU-Mecatron/dvl-a50-ros-driver.git dvl_a50_ros_driver
cd ~/auv_ws
colcon build --packages-select dvl_a50_ros_driver
```

## Configuration
Edit `params/dvl_params.yaml` to configure DVL settings:
- **Network**: TCP/IP address and port
- **Topics**: Raw data and output topic names
- **Frame ID**: TF frame identifier for DVL sensor
- **Covariance**: Choose between DVL's native covariance, FOM-based, or custom values

## Usage

### Launch DVL Driver
```bash
ros2 launch dvl_a50_ros_driver launch_dvl.launch.py
```

This launches two nodes:
1. **publisher** - Connects to DVL hardware via TCP/IP, publishes raw JSON data
2. **dvl_republisher** - Converts raw JSON to TwistWithCovarianceStamped messages

### Verify Operation
Check published topics:
```bash
ros2 topic list
ros2 topic echo /dvl/twist_stamped
```

Call DVL services:
```bash
ros2 service call /dvl/reset_dead_reckoning std_srvs/srv/Trigger
ros2 service call /dvl/toggle std_srvs/srv/SetBool "{data: true}"
```

## Architecture

### Nodes

**Publisher Node** (`/dvl/publisher`)
- Maintains TCP/IP connection to DVL hardware
- Publishes raw JSON data on `/dvl/raw_data`
- Provides services: `reset_dead_reckoning`, `calibrate_gyro`, `get_config`, `toggle`

**DVL Republisher Node** (`/dvl/dvl_republisher`)
- Subscribes to raw JSON data
- Converts to ROS2 standard messages with proper frame conversions (FRD → FLU)
- Publishes TwistWithCovarianceStamped on `/dvl/twist_stamped`
- Handles covariance management with three priority modes

### Topics
- `/dvl/raw_data` (std_msgs/String): Raw JSON from DVL
- `/dvl/twist_stamped` (geometry_msgs/TwistWithCovarianceStamped): Velocity with covariance

### Services
- `/dvl/reset_dead_reckoning` (std_srvs/Trigger): Reset DVL dead reckoning
- `/dvl/calibrate_gyro` (std_srvs/Trigger): Calibrate gyroscope
- `/dvl/get_config` (std_srvs/Trigger): Retrieve DVL configuration
- `/dvl/toggle` (std_srvs/SetBool): Enable/disable acoustic measurements

## Protocol Reference
For DVL protocol details, see: [DVL Protocol Documentation](https://waterlinked.github.io/docs/dvl/dvl-protocol/)
