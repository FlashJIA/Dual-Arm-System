import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import yaml  # Needed for writing files
import os

# TF libraries
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

# Clustering algorithm library
from sklearn.cluster import DBSCAN

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscription = self.create_subscription(
            PointCloud2, '/points', self.listener_callback, 10
        )
        self.get_logger().info('Detection node started: Looking for 5 Red + 5 Blue ...')
        
        # State flag: prevent multiple saves
        self.detection_done = False

    def listener_callback(self, msg):
        # If detection is already done, return immediately (or destroy subscription)
        if self.detection_done:
            return

        # 1. Coordinate transformation
        try:
            if not self.tf_buffer.can_transform('world', msg.header.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
                return
            transform = self.tf_buffer.lookup_transform('world', msg.header.frame_id, rclpy.time.Time())
            msg_world = do_transform_cloud(msg, transform)
        except TransformException:
            return

        # 2. Read PointCloud
        gen = pc2.read_points(msg_world, field_names=("x", "y", "z"), skip_nans=True)
        raw_list = list(gen)
        if len(raw_list) == 0: return

        # Numpy fix (handle structured arrays)
        all_points = np.array(raw_list)
        if all_points.ndim == 1 and all_points.dtype.names:
            all_points = np.column_stack((all_points['x'], all_points['y'], all_points['z']))

        # 3. Filter (keep only objects on the table)
        mask = (all_points[:, 2] > 0.32) & (all_points[:, 2] < 0.5) & \
               (all_points[:, 0] > -0.5) & (all_points[:, 0] < 1.5) & \
               (all_points[:, 1] > -0.5) & (all_points[:, 1] < 0.5)
        scene_points = all_points[mask]
        if len(scene_points) == 0: return

        # 4. Separate Left/Right
        left_mask = scene_points[:, 0] < 0.51
        right_mask = scene_points[:, 0] >= 0.51

        # 5. Cluster to get coordinates
        red_cubes = self.get_centroids(scene_points[left_mask])
        blue_cylinders = self.get_centroids(scene_points[right_mask])

        # 6. Check if all found (5 + 5)
        if len(red_cubes) == 5 and len(blue_cylinders) == 5:
            self.get_logger().info(f'Detection successful! Red cubes: {len(red_cubes)}, Blue cylinders: {len(blue_cylinders)}')
            
            # Save data
            self.save_to_yaml(red_cubes, blue_cylinders)
            
            # Mark as done and stop subscription (save resources)
            self.detection_done = True
            self.destroy_subscription(self.subscription)
            self.get_logger().info('Subscription stopped, perception task finished.')
        else:
            self.get_logger().info(f'Detecting... Currently found: Red={len(red_cubes)}, Blue={len(blue_cylinders)}', throttle_duration_sec=1.0)

    def get_centroids(self, points):
        if len(points) < 10: return []
        clustering = DBSCAN(eps=0.03, min_samples=10).fit(points)
        labels = clustering.labels_
        unique_labels = set(labels)
        centroids = []
        for label in unique_labels:
            if label == -1: continue
            obj_indices = (labels == label)
            # Calculate centroid and convert to standard Python float list (for YAML storage)
            center = np.mean(points[obj_indices], axis=0).tolist()
            centroids.append(center)
        # Sort by Y axis (usually left to right depending on frame)
        centroids.sort(key=lambda p: p[1], reverse=True)
        return centroids

 
    def save_to_yaml(self, red_list, blue_list):
            # ▼▼▼ Fix: Get the directory path of the current .py script ▼▼▼
            script_dir = os.path.dirname(os.path.abspath(__file__))
            
            # Join filenames (this ensures the file is generated in the scripts folder)
            file_path = os.path.join(script_dir, 'detected_objects.yaml')
            # ▲▲▲ Fix end ▲▲▲
            
            data = {
                'red_cubes': red_list,
                'blue_cylinders': blue_list
            }
            
            with open(file_path, 'w') as f:
                yaml.dump(data, f)
                
            self.get_logger().info(f'Coordinates saved to file: {file_path}')
            print("-" * 30)
            print(yaml.dump(data)) 
            print("-" * 30)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()