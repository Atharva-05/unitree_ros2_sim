import yaml
import numpy as np
from PIL import Image

import os
from ament_index_python.packages import get_package_share_directory

def load_map(yaml_file):
    with open(yaml_file, 'r') as file:
        map_data = yaml.safe_load(file)
    
    image_path = os.path.join(get_package_share_directory('go1_navigation') , 'map', map_data['image']) 
    resolution = map_data['resolution']
    origin = map_data['origin']
    negate = map_data['negate']
    occupied_thresh = map_data['occupied_thresh']
    free_thresh = map_data['free_thresh']

    image = Image.open(image_path)
    image = image.convert('L')  # Convert to grayscale
    image_array = np.array(image)

    if negate:
        image_array = np.invert(image_array)

    image_array = np.flipud(image_array)  # Flip to match ROS coordinate system

    return image_array, resolution, origin, occupied_thresh, free_thresh

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import time
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class MapPublisher(Node):
    def __init__(self, yaml_file):
        super().__init__('map_publisher')
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', qos_profile)
        self.map_data, self.resolution, self.origin, self.occupied_thresh, self.free_thresh = load_map(yaml_file)
        self.publish_map()

    def publish_map(self):
        map_msg = OccupancyGrid()
        map_msg.header = Header()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'

        map_msg.info = MapMetaData()
        map_msg.info.map_load_time = self.get_clock().now().to_msg()
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.map_data.shape[1]
        map_msg.info.height = self.map_data.shape[0]
        map_msg.info.origin.position.x = self.origin[0]
        map_msg.info.origin.position.y = self.origin[1]
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        flat_data = self.map_data.flatten()
        flat_data = np.where(flat_data > (self.occupied_thresh * 255), 100, flat_data)
        flat_data = np.where(flat_data < (self.free_thresh * 255), 0, flat_data)
        flat_data = np.where((flat_data != 0) & (flat_data != 100), -1, flat_data)

        map_msg.data = flat_data.tolist()

        while rclpy.ok():
            self.publisher_.publish(map_msg)
            self.get_logger().info('Map published')
            time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    yaml_file = os.path.join(get_package_share_directory('go1_navigation'), 'map', 'map.yaml')
    # yaml_file = '/home/atharvag/ros2_ws/src/go1_sim/go1_navigation/map/map.yaml'
    map_publisher = MapPublisher(yaml_file)
    rclpy.spin(map_publisher)

    map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()