# Water Linked DVL A50 - ROS1 Package

> :warning: This library is for ROS1 and is no longer maintained. ROS2 drivers are available by third parties by searching [github](https://github.com/search?q=ros2%20dvl-a50&type=repositories)

A ROS package for the Water Linked DVL A50. Along with a subscribing client for easy visualization of the communication through ROS.

Water Linked A50 is, by far, the world's smallest commercially available Doppler Velocity Log. With the record-breaking 5 cm min altitude measurability, the A50 is extremely useful for working with tools close to the seabed.

![Image of Water Linked A50](img/DSC04478_1600_web.jpg?raw=true "Water Linked DVL A50")

### Prerequisites
The package has been tested with ROS Noetic, Ubuntu 20.04. While we recommend the C++ version, the Python version of the code has been adjusted and tested to work with all Python >=3.6. For C++, install the following dependencies:
```bash
sudo apt-get install nlohmann-json3-dev
```

## Installation
Assuming you created your catkin workspace at the default location. And have git installed. The below steps should work:
```bash
cd ~/catkin_ws/src
git clone -b master git@github.com:NTU-Mecatron/dvl-a50-ros-driver.git
cd ~/catkin_ws
catkin_make
```

### Usage
Find the DVLs IP address. Once that's done, the package and it's components can be run by following these steps:

**To run the publisher that listens to the TCP port and sends the data to ROS**
```bash
roslaunch dvl_a50_ros_driver launch_dvl.launch
```

The default IP address is 192.168.194.95 which is always made available by the a50 be default. To replace IP of the DVL, specify the argument "ip":

**To run the publisher that listens to the TCP port, displays the raw data in the DVL and sends the data to ROS**
```bash
rosrun dvl_a50_ros_driver publisher.py _ip:=192.168.2.95 _do_log_raw_data:=true
```

**To run a subscriber node that listens to the DVL topic. Helpful for debugging or checking if everything is running as it should be. Choose between "subscriber_gui.py" and "subscriber.py". The GUI makes reading data visually much easier. While the non-GUI version makes it easier to read through the code to see how you can implement code yourself.**
```bash
rosrun dvl_a50_ros_driver subscriber_gui.py
```
![GUI Subscriber](img/a50_gui.png?raw=true "Interface as seen when running the GUI version of the subscriber")

## Documentation
The node publishes data to the topics: "*dvl/json_data*" and "*dvl/data*".
* *dvl/json_data*: uses a simple String formated topic that publishes the raw json data coming from the DVL.
* *dvl/data*: Uses a custom message type that structures the parsed data following our protocol. Read more about the protocol here: [DVL Protocol](https://waterlinked.github.io/docs/dvl/dvl-protocol/)

![rqt_graph of the package in action](img/a50_graph.png?raw=true "Graph of the package's node-to-node structure")

*The graph illustrates the topics and nodes created when the package is run.*
