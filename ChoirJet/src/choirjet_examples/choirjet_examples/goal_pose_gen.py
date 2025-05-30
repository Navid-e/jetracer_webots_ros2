import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose, PointStamped

from tf2_ros import TransformException
from tf2_ros import TransformListener, Buffer
from builtin_interfaces.msg import Duration
from tf2_geometry_msgs import do_transform_pose
from nav_msgs.msg import Path

class GoalPoseNode(Node):
    def __init__(self):
        super().__init__('goal_pose_gen')

        self.goal_pose_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        self.plan_subscription = self.create_subscription(
            Path,
            '/plan0',
            self.plan_callback,
            10
        )
        self.goal_point_publisher = self.create_publisher(
            PointStamped,
            '/goal_point',
            10
        )

        self.tf_buffer = Buffer(Duration(sec=10, nanosec=0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_goal_pose = None

        self.proximity = 0.1 # Specify the proximity for progressing from each point to the next
        self.brake_proximity = 0.01

        self.timer = self.create_timer(0.1, self.controller_callback)  # Update control commands every 0.1 seconds

    def plan_callback(self, plan_msg):
        if plan_msg.poses:
            self.current_goal_pose = self.calculate_current_goal_pose(plan_msg)
            self.publish_current_goal_pose()

    def get_current_pose(self):
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                'map', 'ack_link', rclpy.time.Time()
            )
            pose_stamped = PoseStamped()
            pose_stamped.header = transform_stamped.header
            pose_stamped.pose = Pose(
                position=Point(
                    x=transform_stamped.transform.translation.x,
                    y=transform_stamped.transform.translation.y,
                    z=transform_stamped.transform.translation.z,
                ),
                orientation=Quaternion(
                    x=transform_stamped.transform.rotation.x,
                    y=transform_stamped.transform.rotation.y,
                    z=transform_stamped.transform.rotation.z,
                    w=transform_stamped.transform.rotation.w,
                ),
            )
            return pose_stamped
        except TransformException as e:
            self.get_logger().error('Failed to lookup transform: %s' % str(e))
            return None

    def calculate_current_goal_pose(self, plan_msg):
        current_pose = self.get_current_pose()
        if current_pose is None:
            return None

        min_distance = float('inf')
        nearest_point = None

        for pose_stamped in plan_msg.poses:
            dx = pose_stamped.pose.position.x - current_pose.pose.position.x
            dy = pose_stamped.pose.position.y - current_pose.pose.position.y
            distance = math.sqrt(dx ** 2 + dy ** 2)

            if distance < min_distance:
                min_distance = distance
                nearest_point = pose_stamped.pose

        if nearest_point is None:
            return None

        
        # Reached the nearest point, progress to the next point
        nearest_index = next(
            (i for i, pose in enumerate(plan_msg.poses) if pose.pose == nearest_point), None
        )
        if nearest_index is not None and nearest_index < len(plan_msg.poses) - 1:
            return plan_msg.poses[nearest_index + 5].pose

        return nearest_point

    def publish_current_goal_pose(self):
        if self.current_goal_pose is not None:
            goal_pose_msg = PoseStamped()
            goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
            goal_pose_msg.header.frame_id = 'map'
            goal_pose_msg.pose = self.current_goal_pose
            self.goal_pose_publisher.publish(goal_pose_msg)

    def controller_callback(self):
        # Your control logic here
        pass

    def publish_current_goal_pose(self):
        if self.current_goal_pose is not None:
            goal_pose_msg = PoseStamped()
            goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
            goal_pose_msg.header.frame_id = 'map'
            goal_pose_msg.pose = self.current_goal_pose

            # Create a PointStamped message
            goal_point_msg = PointStamped()
            goal_point_msg.header.stamp = goal_pose_msg.header.stamp
            goal_point_msg.header.frame_id = goal_pose_msg.header.frame_id
            goal_point_msg.point = self.current_goal_pose.position

            # Publish both the goal pose and the goal point
            self.goal_pose_publisher.publish(goal_pose_msg)
            self.goal_point_publisher.publish(goal_point_msg)

def main(args=None):
    rclpy.init(args=args)
    goal_pose_gen = GoalPoseNode()
    rclpy.spin(goal_pose_gen)
    goal_pose_gen.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
