import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

import numpy as np
import cv2


class TimedImageSaver(Node):
    def __init__(self):
        super().__init__('timed_image_saver_node')

        # Setup timestamped folder
        timestamp = datetime.now().strftime("%H_%M_%S_%d_%m_%Y")
        desktop_path = os.path.join(os.path.expanduser("~"), "Desktop")
        folder_name = f"captures_{timestamp}"
        self.capture_folder = os.path.join(desktop_path, folder_name)
        os.makedirs(self.capture_folder, exist_ok=True)
        self.get_logger().info(f"[INFO] Saving images to: {self.capture_folder}")

        # Subscribe to image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            '/drone/image/compressed',
            self.image_callback,
            10
        )

        self.latest_image = None  # Holds latest decoded OpenCV image

        # Timer: Save image every 0.5 seconds
        self.timer = self.create_timer(0.5, self.save_image)

    def image_callback(self, msg):
        # Decode the compressed image and store it
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if image is not None:
            self.latest_image = image
        else:
            self.get_logger().warn("Failed to decode image!")

    def save_image(self):
        if self.latest_image is not None:
            image_time = datetime.now().strftime("%H_%M_%S_%d_%m_%Y")
            filename = os.path.join(self.capture_folder, f"img_{image_time}.jpg")
            cv2.imwrite(filename, self.latest_image)
            self.get_logger().info(f"[SAVED] {filename}")
        else:
            self.get_logger().info("[WAITING] No image received yet.")


def main(args=None):
    rclpy.init(args=args)
    node = TimedImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[EXIT] Capture stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
