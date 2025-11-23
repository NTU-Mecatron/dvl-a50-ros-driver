#!/usr/bin/env python3
import socket
import json
import rclpy
from rclpy.node import Node
from time import sleep
from std_msgs.msg import String
from dvl_a50_ros_driver.msg import DVL
from dvl_a50_ros_driver.msg import DVLBeam
import select

class DVLA50Publisher(Node):
    def __init__(self):
        super().__init__('dvl_a50_publisher')
        
        # Declare and get parameters
        self.declare_parameter('ip', '10.42.0.186')
        self.declare_parameter('port', 16171)
        self.declare_parameter('do_log_raw_data', False)
        
        self.TCP_IP = self.get_parameter('ip').value
        self.TCP_PORT = self.get_parameter('port').value
        self.do_log_raw_data = self.get_parameter('do_log_raw_data').value
        
        # Initialize socket
        self.s = None
        self.oldJson = ""
        
        # Create publishers
        self.pub_raw = self.create_publisher(String, 'dvl/json_data', 10)
        self.pub = self.create_publisher(DVL, 'dvl/data', 10)
        
        # Initialize DVL message objects
        self.theDVL = DVL()
        self.beam0 = DVLBeam()
        self.beam1 = DVLBeam()
        self.beam2 = DVLBeam()
        self.beam3 = DVLBeam()
        
        # Connect to DVL
        self.connect()
        
        # Create timer for periodic data collection (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def connect(self):
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect((self.TCP_IP, self.TCP_PORT))
            self.s.settimeout(1)
        except socket.error as err:
            self.get_logger().error(f"No route to host, DVL might be booting? {err}")
            sleep(1)
            self.connect()
    
    def getData(self):
        raw_data = ""
        
        while not '\n' in raw_data:
            try:
                rec = self.s.recv(1)
                if len(rec) == 0:
                    self.get_logger().error("Socket closed by the DVL, reopening")
                    self.connect()
                    continue
            except socket.timeout as err:
                self.get_logger().error(f"Lost connection with the DVL, reinitiating the connection: {err}")
                self.connect()
                continue
            raw_data = raw_data + rec.decode()
        
        raw_data = self.oldJson + raw_data
        self.oldJson = ""
        raw_data = raw_data.split('\n')
        self.oldJson = raw_data[1]
        raw_data = raw_data[0]
        return raw_data
    
    def timer_callback(self):
        raw_data = self.getData()
        data = json.loads(raw_data)
        
        # edit: the logic in the original version can't actually publish the raw data
        # we slightly change the if else statement so now
        # do_log_raw_data is true: publish the raw data to /dvl/json_data topic, fill in theDVL using velocity data and publish to dvl/data topic
        # do_log_raw_data is false: only fill in theDVL using velocity data and publish to dvl/data topic
        
        if self.do_log_raw_data:
            self.get_logger().info(raw_data)
            msg = String()
            msg.data = raw_data
            self.pub_raw.publish(msg)
            if data["type"] != "velocity":
                return
        else:
            if data["type"] != "velocity":
                return
            msg = String()
            msg.data = raw_data
            self.pub_raw.publish(msg)
        
        self.theDVL.header.stamp = self.get_clock().now().to_msg()
        self.theDVL.header.frame_id = "dvl_link"
        self.theDVL.time = data["time"]
        self.theDVL.velocity.x = data["vx"]
        self.theDVL.velocity.y = data["vy"]
        self.theDVL.velocity.z = data["vz"]
        self.theDVL.fom = data["fom"]
        self.theDVL.altitude = data["altitude"]
        self.theDVL.velocity_valid = data["velocity_valid"]
        self.theDVL.status = data["status"]
        self.theDVL.form = data["format"]
        
        self.beam0.id = data["transducers"][0]["id"]
        self.beam0.velocity = data["transducers"][0]["velocity"]
        self.beam0.distance = data["transducers"][0]["distance"]
        self.beam0.rssi = data["transducers"][0]["rssi"]
        self.beam0.nsd = data["transducers"][0]["nsd"]
        self.beam0.valid = data["transducers"][0]["beam_valid"]
        
        self.beam1.id = data["transducers"][1]["id"]
        self.beam1.velocity = data["transducers"][1]["velocity"]
        self.beam1.distance = data["transducers"][1]["distance"]
        self.beam1.rssi = data["transducers"][1]["rssi"]
        self.beam1.nsd = data["transducers"][1]["nsd"]
        self.beam1.valid = data["transducers"][1]["beam_valid"]
        
        self.beam2.id = data["transducers"][2]["id"]
        self.beam2.velocity = data["transducers"][2]["velocity"]
        self.beam2.distance = data["transducers"][2]["distance"]
        self.beam2.rssi = data["transducers"][2]["rssi"]
        self.beam2.nsd = data["transducers"][2]["nsd"]
        self.beam2.valid = data["transducers"][2]["beam_valid"]
        
        self.beam3.id = data["transducers"][3]["id"]
        self.beam3.velocity = data["transducers"][3]["velocity"]
        self.beam3.distance = data["transducers"][3]["distance"]
        self.beam3.rssi = data["transducers"][3]["rssi"]
        self.beam3.nsd = data["transducers"][3]["nsd"]
        self.beam3.valid = data["transducers"][3]["beam_valid"]
        
        self.theDVL.beams = [self.beam0, self.beam1, self.beam2, self.beam3]
        
        self.pub.publish(self.theDVL)
    
    def destroy_node(self):
        if self.s:
            self.s.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    publisher = DVLA50Publisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
