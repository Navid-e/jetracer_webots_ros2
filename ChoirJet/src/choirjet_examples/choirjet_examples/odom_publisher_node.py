import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, Quaternion
from nav_msgs.msg import Odometry
from rclpy.time import Time
import tf2_ros

class OdomPublisherNode(Node):

    def __init__(self):
        super().__init__('odom_publisher')
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.timer = self.create_timer(1.0, self.publish_odom)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.current_pose = Pose()

    def publish_odom(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        try:
            transform = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time())
            self.current_pose = transform.transform.translation

            # Populate pose information
            odom_msg.pose.pose.position.x = self.current_pose.x
            odom_msg.pose.pose.position.y = self.current_pose.y
            odom_msg.pose.pose.position.z = self.current_pose.z

            
            odom_msg.pose.covariance[0] = 0.0
            # Set covariance values as needed

            self.odom_publisher.publish(odom_msg)

        except tf2_ros.LookupException as e:
            self.get_logger().warn(f"TF Lookup Exception: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    odom_publisher_node = OdomPublisherNode()
    rclpy.spin(odom_publisher_node)
    odom_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
