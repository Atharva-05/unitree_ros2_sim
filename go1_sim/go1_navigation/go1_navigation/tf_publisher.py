import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile


class OdomToTFBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_to_tf_broadcaster')
        
        # Initialize the TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create a subscription to the /odom topic
        qos_profile = QoSProfile(depth=10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile)
        
        self.get_logger().info('Odom to TF Broadcaster started')
    
    def odom_callback(self, msg: Odometry):
        # Create a TransformStamped message
        t = TransformStamped()
        
        # Set the timestamp to the time of the received message
        t.header.stamp = msg.header.stamp
        
        # Set the frame IDs
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        
        # Set the translation
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        # Set the rotation
        t.transform.rotation = msg.pose.pose.orientation

        t_map = TransformStamped()
        t_map.header.stamp = msg.header.stamp
        t_map.header.frame_id = "map"
        t_map.child_frame_id = "odom"
        t_map.transform.translation.x = 0.0
        t_map.transform.translation.y = 0.0
        t_map.transform.translation.z = 0.0
        t_map.transform.rotation.x = 0.0
        t_map.transform.rotation.y = 0.0
        t_map.transform.rotation.z = 0.0
        t_map.transform.rotation.w = 1.0

        # Broadcast the transforms
        # Fixed map -> odom transform
        # odom -> base link based on ground truth plugin
        self.tf_broadcaster.sendTransform(t)
        self.tf_broadcaster.sendTransform(t_map)

        # self.get_logger().info('Broadcasting transform from {} to {}'.format(
        #     t.header.frame_id, t.child_frame_id))


def main(args=None):
    rclpy.init(args=args)
    
    node = OdomToTFBroadcaster()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()