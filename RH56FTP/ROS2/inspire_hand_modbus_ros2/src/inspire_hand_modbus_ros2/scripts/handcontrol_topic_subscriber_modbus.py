#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HandControlSubscriber(Node):
    def __init__(self):
        super().__init__("handcontrol_subscriber")
        self.subscription = self.create_subscription(
            String,
            "touch_data",
            self.listener_callback,
            10
        )
        self.subscription  

    def listener_callback(self, msg):
        self.get_logger().info(f"接收到触觉数据：\n{msg.data}")


def main(args=None):
    rclpy.init(args=args)
    subscriber = HandControlSubscriber()
    
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        subscriber.get_logger().info("手动停止节点")
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

