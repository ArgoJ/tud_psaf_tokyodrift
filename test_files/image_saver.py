import rclpy
import argparse
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import datetime
import os


class ImageSaver(Node):
    def __init__(self, path: str = None, image_msg: str = '/camera/realsense2_camera_node/color/image_raw') -> None:
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            image_msg,
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.image_saved = False
        self.path = os.getcwd() if path is None else os.path.join(os.getcwd(), path)

        os.makedirs(self.path, exist_ok=True)
        print(self.path)


    def image_callback(self, msg):
        if not self.image_saved:
            self.get_logger().info('Image received, saving to image.png')
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                filename = f"image_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
                cv2.imwrite(os.path.join(self.path, filename), cv_image)
                self.image_saved = True
                self.get_logger().info('Image saved successfully!')
                rclpy.shutdown()  # Automatically stop the node
            except Exception as e:
                self.get_logger().error(f'Failed to save image: {e}')

def main():
    parser = argparse.ArgumentParser(description='ROS2 Image Saver')
    parser.add_argument('--path', type=str, help='Path to save the image', default=os.getcwd())
    args = parser.parse_args()

    rclpy.init()
    node = ImageSaver(args.path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
