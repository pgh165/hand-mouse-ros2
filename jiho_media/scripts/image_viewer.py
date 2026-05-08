#!/usr/bin/env python3
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

os.environ.setdefault("DISPLAY", ":0")


class ImageViewer(Node):

    def __init__(self):
        super().__init__("image_viewer")
        self.declare_parameter("topic", "hand_mouse/image/compressed")
        topic = self.get_parameter("topic").value
        self.create_subscription(CompressedImage, topic, self._cb, 10)
        self._window = topic.replace("/", "_")
        self.get_logger().info(f"ImageViewer — topic={topic}")

    def _cb(self, msg: CompressedImage):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is not None:
            cv2.imshow(self._window, frame)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
