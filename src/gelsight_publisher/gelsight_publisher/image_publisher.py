import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
from gelsight_publisher.utilities.gelsightmini import GelSightMini
from gelsight_publisher.config import ConfigModel, GSConfig


class GelSightImagePublisher(Node):
    def __init__(self, config: ConfigModel):
        super().__init__('gelsight_image_publisher')
        self.config = config
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(Image, 'gelsight/image', 10)

        self.cam = GelSightMini(config.camera_width, config.camera_height)
        self.cam.select_device(config.default_camera_index)
        self.cam.start()

        self.timer = self.create_timer(1.0 / config.max_fps, self.publish_image)

    def publish_image(self):
        frame = self.cam.update(dt=0)
        if frame is None:
            return

        # frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


def main(args=None):
    from gelsight_publisher.config import GSConfig
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--gs-config', type=str, default='default_config.json')
    args = parser.parse_args()

    gs_config = GSConfig(args.gs_config)
    gs_config.config.pointcloud_enabled = False

    rclpy.init()
    node = GelSightImagePublisher(gs_config.config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cam.camera:
            node.cam.camera.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
