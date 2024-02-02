import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandsSubscriberNode(Node):
    def __init__(self):
        super().__init__('commands_subscriber')

        # ROS setup
        self.subscription = self.create_subscription(String, 'commands_topic', self.callback, 10)

    def callback(self, msg):
        # ROS callback for receiving commands
        self.get_logger().info(f"Received command: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = CommandsSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


