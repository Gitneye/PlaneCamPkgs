import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import zmq
from threading import Thread
import sys

class PlaneDataTelemetryNode(Node):
    def __init__(self):
        super().__init__('plane_data_node')

        self.data_subscriber = self.create_subscription(String, 
                'plane_data_topic', self.data_callback, 10)

        context = zmq.Context()

        # Data socket creation
        self.data_socket = context.socket(zmq.PUB)
        self.data_socket.bind("tcp://192.168.1.150:5002")

    def data_callback(self, msg):
        # Process 'plane_data' message and send it to the client
        data = msg.data
        self.get_logger().info("Sending plane data")
      
        self.data_socket.send_string(data)
        print("Data sent")

class CommandReceiverNode(Node):
    def __init__(self):
        super().__init__('command_receiver')

        self.command_thread = Thread(target=self.receive_commands)
        self.command_thread.start()

        self.command_publisher = self.create_publisher(String,
                'commands_topic', 10)

    def receive_commands(self):
        print("Command thread started")
        context = zmq.Context()
        self.command_socket = context.socket(zmq.REP)
        self.command_socket.bind("tcp://192.168.1.150:5001")

        while True:
            print("Run loop")
            #Add timeout to socket 
            data = self.command_socket.recv_string()
            print(data)
            
            # Process received command data
            self.command_publisher.publish(String(data=data))

            # Send acknowledgment back to the client
            self.command_socket.send_string("Command received")

class SystemDataPublisherNode(Node):
    def __init__(self):
        super().__init__('system_data_publisher')

        # ROS setup
        self.subscription = self.create_subscription(String, 'system_data', 
                self.callback,10)

        # ZeroMQ setup for data publishing
        context = zmq.Context()
        self.zmq_socket = context.socket(zmq.PUB)
        self.zmq_socket.bind("tcp://192.168.1.150:5000")

    def callback(self, msg):

        system_data = "Sample System Data"
        print("Publishing system data")
        self.zmq_socket.send_string(system_data)
       


def main(args=None):
    rclpy.init(args=args)

    #Create instances of the nodes
    plane_data_node = PlaneDataTelemetryNode()
    command_receiver = CommandReceiverNode()
    system_data_node = SystemDataPublisherNode()

    #Use MultiThreadedExecutor to handle all nodes in seperate threads
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
    executor.add_node(plane_data_node)
    executor.add_node(command_receiver)
    executor.add_node(system_data_node)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        plane_data_node.destroy_node()
        command_receiver.destroy_node()
        system_data_node.destroy_node()
        rclpy.shutdown()

