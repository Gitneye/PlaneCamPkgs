import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import cv2
import numpy as np

class PlaneDataPublisherNode(Node):
    def __init__(self):
        super().__init__('plane_data_publisher')

        # ROS setup
        self.publisher = self.create_publisher(String, 'plane_data_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_data)

    def create_square_image(self):
        image_size = (300, 300)
        square_color = (0, 255, 0)  # Green color for the square
        image = np.zeros((image_size[0], image_size[1], 3), dtype=np.uint8)
        square_size = 100
        square_start = (image_size[0] // 2 - square_size // 2, image_size[1] // 2 - square_size // 2)
        square_end = (square_start[0] + square_size, square_start[1] + square_size)
        cv2.rectangle(image, square_start, square_end, square_color, thickness=-1)  # Filled rectangle
        return image

    def generate_adsb_data(self):
        # Generate sample ADS-B data as a dictionary
        adsb_data = {
            'callsign': 'ABC123',
            'altitude': 35000,
            'latitude': 37.7749,
            'longitude': -122.4194,
            'velocity': 500
        }
        return adsb_data

    def publish_data(self):
        # Timer callback to publish image and ADS-B data as a JSON structure
        image = self.create_square_image()

        adsb_data = self.generate_adsb_data()

        data_to_publish = {
            'image': image.tolist(),  # Convert NumPy array to a nested Python list
            'adsb_data': adsb_data
        }

        json_data = json.dumps(data_to_publish)

        msg = String()
        msg.data = json_data

        self.publisher.publish(msg)
        self.get_logger().info("Published data to 'plane_data_topic'")

def main(args=None):
    rclpy.init(args=args)
    node = PlaneDataPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
