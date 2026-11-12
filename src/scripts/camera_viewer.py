# Simple camera viewer to see what the robot sees
# Press 'q' to quit, 's' to save screenshot

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

# check if opencv is installed
try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.subscription = self.create_subscription(
            Image, '/camera', self.image_callback, 10)
        self.frame_count = 0
        self.get_logger().info("Camera viewer started, waiting for images...")
        if not HAS_CV2:
            self.get_logger().warn("OpenCV not found. Install with: pip install opencv-python")

    def image_callback(self, msg):
        self.frame_count += 1

        # if opencv isn't installed just log frames
        if not HAS_CV2:
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f"Received {self.frame_count} frames ({msg.width}x{msg.height})")
            return

        try:
            # convert ROS image to opencv
            if msg.encoding == 'rgb8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgr8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
            else:
                self.get_logger().warn(f"Unknown encoding: {msg.encoding}")
                return

            # show frame number on image
            cv2.putText(img, f"Frame: {self.frame_count}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.imshow("Robot Camera", img)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                rclpy.shutdown()
            elif key == ord('s'):
                filename = f"screenshot_{self.frame_count}.png"
                cv2.imwrite(filename, img)
                self.get_logger().info(f"Saved {filename}")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if HAS_CV2:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
