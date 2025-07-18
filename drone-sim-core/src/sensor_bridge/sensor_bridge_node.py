import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2

# Gazebo Transport
import gz.transport
import gz.msgs
import numpy as np

class SensorBridgeNode(Node):
    def __init__(self):
        super().__init__('sensor_bridge_node')
        self.publisher_image = self.create_publisher(Image, '/camera/image_raw', 10)
        self.publisher_lidar = self.create_publisher(PointCloud2, '/lidar/points', 10)
        # Gazebo Transport Node
        self.gz_node = gz.transport.Node()
        self.gz_node.subscribe('/camera/image_raw', self.camera_callback)
        self.gz_node.subscribe('/lidar/points', self.lidar_callback)
        self.get_logger().info('SensorBridgeNode started, bridging Gazebo topics to ROS 2.')

    def camera_callback(self, msg):
        # TODO: gz.msgs.Image -> sensor_msgs/Image 変換実装
        ros_img = Image()
        ros_img.header.stamp = self.get_clock().now().to_msg()
        ros_img.header.frame_id = 'camera_link'
        # プレースホルダ: 実際の変換はgz.msgs.Imageの仕様に合わせて実装
        self.publisher_image.publish(ros_img)

    def lidar_callback(self, msg):
        # TODO: gz.msgs.PointCloudPacked -> sensor_msgs/PointCloud2 変換実装
        ros_pc = PointCloud2()
        ros_pc.header.stamp = self.get_clock().now().to_msg()
        ros_pc.header.frame_id = 'lidar_link'
        # プレースホルダ: 実際の変換はgz.msgs.PointCloudPackedの仕様に合わせて実装
        self.publisher_lidar.publish(ros_pc)

def main(args=None):
    rclpy.init(args=args)
    node = SensorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 