#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dvl_a50_ros_driver.msg import DVL
from dvl_a50_ros_driver.msg import DVLBeam


class DVLA50Subscriber(Node):
    def __init__(self):
        super().__init__('dvl_a50_subscriber')
        
        # Create subscriptions
        self.raw_subscription = self.create_subscription(
            String,
            'dvl/json_data',
            self.callbackRAW,
            10)
        self.dvl_subscription = self.create_subscription(
            DVL,
            'dvl/data',
            self.callback,
            10)
    
    def callbackRAW(self, data):
        self.get_logger().info(f'Data received: {data.data}')
    
    def callback(self, data):
        self.get_logger().info(f'Time received: {data.time}')


def main(args=None):
    rclpy.init(args=args)
    subscriber = DVLA50Subscriber()
    
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
