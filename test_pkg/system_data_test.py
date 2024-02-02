import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SystemDataPublisherNode(Node):
    def __init__(self):
        super().__init__('system_data_publisher')

        # ROS setup
        self.publisher = self.create_publisher(String, 'system_data', 10)
        self.timer = self.create_timer(1.0, self.publish_system_status)

    def publish_system_status(self):
        # Timer callback to publish system status message
        msg = String()
        msg.data = "System status: OK"
        self.publisher.publish(msg)
        self.get_logger().info("Published system status: OK")

def main(args=None):
    rclpy.init(args=args)
    node = SystemDataPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

