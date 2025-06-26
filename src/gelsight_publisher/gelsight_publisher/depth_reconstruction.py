import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np


from gelsight_publisher.config import GSConfig
from gelsight_publisher.utilities.image_processing import (
    apply_cmap, color_map_from_txt, normalize_array, trim_outliers
)
from gelsight_publisher.utilities.reconstruction import Reconstruction3D


from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

class DepthReconstructionNode(Node):
    def __init__(self, config):
        super().__init__('depth_reconstruction_node')
        self.config = config
        self.bridge = CvBridge()

        # Load colormap
        # self.cmap = color_map_from_txt(config.cmap_txt_path, config.cmap_in_BGR_format)
        self.cmap = color_map_from_txt(config.cmap_txt_path, False)
        

        # Load NN
        self.reconstruction = Reconstruction3D(config.camera_width, config.camera_height, config.use_gpu)
        if self.reconstruction.load_nn(config.nn_model_path) is None:
            self.get_logger().error("Failed to load neural network")
            exit(1)

        # Sub and pub
        self.sub = self.create_subscription(Image, 'gelsight/image', self.image_callback, 1)
        self.pub = self.create_publisher(Image, 'gelsight/depth', 1)
        self.pcd_pub = self.create_publisher(PointCloud2, 'gelsight/pointcloud', 1)
        self.pixel_mm_scale = config.pixel_mm_scale  # e.g. 0.0634 (mm/pixel)
        self.image_buffer = []
        self.buffer_size = 3  # You can adjust this
    

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:
            self.get_logger().error(f"CVBridge conversion failed: {e}")
            return
        
        # self.image_buffer.append(image)
        # if len(self.image_buffer) > self.buffer_size:
        #     self.image_buffer.pop(0)
            
        # avg_image = np.mean(self.image_buffer, axis=0).astype(np.uint8)

        # Depth map from reconstruction
        depth_map, contact_mask, grad_x, grad_y = self.reconstruction.get_depthmap(
            image=image,
            markers_threshold=(self.config.marker_mask_min, self.config.marker_mask_max),
        )
        if np.isnan(depth_map).any():
            self.get_logger().warn("Depth map contains NaNs. Skipping frame.")
            return

        # Normalize and color-map depth
        depth_trim = trim_outliers(depth_map, 1, 100)
        depth_norm = normalize_array(depth_trim, min_divider=10)
        depth_rgb = apply_cmap(data=depth_norm, cmap=self.cmap).astype(np.uint8)
    
        

        
        # depth_gray = (depth_norm * 255).astype(np.uint8)
        
        # Publish
        depth_msg = self.bridge.cv2_to_imgmsg(depth_rgb, encoding='rgb8')
        # depth_msg = self.bridge.cv2_to_imgmsg(depth_gray, encoding='mono8')
        depth_msg.header = Header()
        depth_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(depth_msg)

        # === Point Cloud Publishing ===
        h, w = depth_norm.shape

        scale = self.pixel_mm_scale  # e.g., 0.0634 mm/pixel

        # Create meshgrid for X and Y
        x_range = np.arange(w)
        y_range = np.arange(h)
        xx, yy = np.meshgrid(x_range, y_range)

        # Flatten and center X and Y
        x_flat = (xx - w / 2).flatten() * scale
        y_flat = (yy - h / 2).flatten() * scale
        z_flat = depth_norm.flatten()

        # Filter out NaNs
        valid_mask = ~np.isnan(z_flat)
        points = np.stack([
            x_flat[valid_mask],
            y_flat[valid_mask],
            z_flat[valid_mask],
        ], axis=1)

        # Define ROS2 PointCloud2 message fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'  # Change if needed

        pc_msg = pc2.create_cloud(header, fields, points.astype(np.float32))
        self.pcd_pub.publish(pc_msg)



def main(args=None):
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--gs-config', type=str, default='default_config.json')
    args = parser.parse_args()

    gs_config = GSConfig(args.gs_config)

    rclpy.init()
    node = DepthReconstructionNode(gs_config.config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
