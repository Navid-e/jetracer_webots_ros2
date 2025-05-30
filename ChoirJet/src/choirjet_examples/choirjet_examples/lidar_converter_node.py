import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

class LidarConverterNode(Node):
    def __init__(self):
        super().__init__('lidar_converter_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/agent_0/agent_0/lidar',
            self.lidar_callback,
            10
        )
        """self.subscription = self.create_subscription(
            Float64,
            '/cmd_vel',
            self.cmd_callback,
            10
        )"""
        
        self.publisher_scan = self.create_publisher(LaserScan, '/scan', 10)
        self.publisher_laser_scan = self.create_publisher(LaserScan, '/laser_scan', 10)
        self.publisher_lidar = self.create_publisher(LaserScan, '/lidar', 10)
    def lidar_callback(self, msg):
        self.publisher_scan.publish(msg)
        self.publisher_laser_scan.publish(msg)
        self.publisher_lidar.publish(msg)
    """def cmd_callback(self, msg):
     vel = msg"""
def main(args=None):
    rclpy.init(args=args)
    lidar_converter_node = LidarConverterNode()
    rclpy.spin(lidar_converter_node)
    lidar_converter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()