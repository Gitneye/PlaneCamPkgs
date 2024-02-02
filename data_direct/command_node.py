import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import zmq
from threading import Thread

class CommandReceiverNode(Node):
    def __init__(self):
        super().__init__('command_receiver')

        self.command_thread = Thread(target=self.receive_commands)
        self.command_thread.start()

        self.command_publisher = self.create_publisher(String,
                'commands_topic', 10)

    def receive_commands(self):
        context = zmq.Context()
        self.command_socket = context.socket(zmq.REP)
        self.command_socket.bind("tcp://192.168.1.150: 5001")
        while True:
            print("Run loop")
            #Add timeout to socket 
            data = self.command_socket.recv_string()
            print(data)
            
            # Process received command data
            #self.command_publisher.publish(String(data=data))

            # Send acknowledgment back to the client
            self.command_socket.send_string("Command received")

def main(args=None):
    rclpy.init(args=args)

    command_receiver = CommandReceiverNode()
    
    try:
        rclpy.spin(command_receiver)
    finally:
        command_receiver.destroy_node()
        rclpy.shutdown()

