from vmbpy import *
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.camera_publisher(String, 'camera_feed', 10)


    def camera_interface(self):
        try:
            with VmbSystem.get_instance() as vmb:
                cams = vmb.get_all_cameras()
                with cams[0] as cam:
                    while True:
                        frame = cam.get_frame()
                        cv_frame = frame.as_opencv_image()
                        cv_frame_scaled = cv2.resize(cv_frame, (640,480)) 
                        self.camera_publisher_.publish(cv_frame_scaled)

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)

    camera_node.destroy_node()
    rclpy.shutdown()
