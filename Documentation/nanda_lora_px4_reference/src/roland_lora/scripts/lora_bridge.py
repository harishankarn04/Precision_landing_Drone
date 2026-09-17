#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from roland_lora_msgs.msg import LoRaRanging
import json

class LoRaBridgeNode(Node):
    def __init__(self):
        super().__init__('lora_bridge_node')
        self.sub = self.create_subscription(String, '/gz/lora/ranging', self.gz_cb, 10)
        self.pub = self.create_publisher(LoRaRanging, '/lora/ranging', 10)
        self.get_logger().info("LoRa JSON Bridge Started.")

    def gz_cb(self, msg):
        try:
            data = json.loads(msg.data)
            out = LoRaRanging()
            out.anchor_id = int(data["anchor_id"])
            out.tag_id = int(data["tag_id"])
            out.range = float(data["range"])
            out.rssi = float(data["rssi"])
            out.snr = float(data["snr"])
            out.range_error = float(data["range_error"])
            self.pub.publish(out)
        except Exception as e:
            self.get_logger().error(f"Error parsing LoRa JSON: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LoRaBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
