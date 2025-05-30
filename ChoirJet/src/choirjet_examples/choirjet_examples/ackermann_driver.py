import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from ackermann_msgs.msg import AckermannDrive
from tf2_ros import TransformException
from tf2_ros import TransformListener, Buffer
from builtin_interfaces.msg import Duration
from tf2_geometry_msgs import do_transform_pose
from nav_msgs.msg import Path
import numpy as np

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('ackermann_driver')

        self.goal_pose_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )

        self.cmd_publisher = self.create_publisher(
            AckermannDrive,
            '/agent_0/cmd_ackermann',
            10
        )

        self.plan_subscription = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10
        )

        self.steering_Kp = 0.2
        self.steering_Ki = 0.0
        self.steering_Kd = 0.0

        self.acceleration_Kp = 0.5
        self.acceleration_Ki = 0.0
        self.acceleration_Kd = 0.0

        self.steering_error_sum = 0.0
        self.steering_last_error = 0.0

        self.acceleration_error_sum = 0.0
        self.acceleration_last_error = 0.0

        self.steering_limit = 0.65558
        self.acceleration_limit = 0.5
        self.steering_error_limit = math.pi
        self.tf_buffer = Buffer(Duration(sec=10, nanosec=0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_pose = None
        self.current_goal_pose = None

        self.proximity = 0.2 #0.3  # Specify the proximity for progressing from each point to the next
        self.brake_proximity = 0.01
        self.proximity_st = 0.2
        self.timer = self.create_timer(0.1, self.controller_callback)  # Update control commands every 0.1 seconds

    def goal_pose_callback(self, goal_pose_msg):
        self.goal_pose = goal_pose_msg.pose

    def plan_callback(self, plan_msg):
        if plan_msg.poses:
            self.current_goal_pose = self.get_nearest_point(plan_msg)
            self.current_goal_pose_st = self.get_nearest_point_st(plan_msg)
    def controller_callback(self):
        current_pose = self.get_current_pose()
        if current_pose is None or self.current_goal_pose is None:
            return

        steering_control = self.calculate_steering_control(current_pose)
        acceleration_control = self.calculate_acceleration_control(current_pose)

        # Limit the steering and acceleration commands within the specified limits
        steering_control = max(min(steering_control, self.steering_limit), -self.steering_limit)
        acceleration_control = max(min(acceleration_control, self.acceleration_limit), -self.acceleration_limit)

        self.publish_control_commands(steering_control, acceleration_control)

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

    def calculate_acceleration_control(self, current_pose):
        
        
            dx = self.goal_pose.position.x - current_pose.pose.position.x
            dy = self.goal_pose.position.y - current_pose.pose.position.y

            # Calculate the distance error
            distance_error = math.sqrt(dx ** 2 + dy ** 2)
            #if distance_error < self.brake_proximity :
            #    distance_error = 0 
            distance_error_diff = distance_error - self.acceleration_last_error
            self.acceleration_error_sum += distance_error
            acceleration_output = (
                self.acceleration_Kp * distance_error +
                self.acceleration_Ki * self.acceleration_error_sum +
                self.acceleration_Kd * distance_error_diff
            )

            self.acceleration_last_error = distance_error
            return acceleration_output
    def calculate_steering_control(self, current_pose):
        base_link_angle = math.atan2(
            2 * (current_pose.pose.orientation.w * current_pose.pose.orientation.z +
                current_pose.pose.orientation.x * current_pose.pose.orientation.y),
            1 - 2 * (current_pose.pose.orientation.y ** 2 + current_pose.pose.orientation.z ** 2)
        )

        #error_vector = [self.goal_pose.position.x - current_pose.pose.position.x,
        #                self.goal_pose.position.y - current_pose.pose.position.y]
        error_vector = [self.current_goal_pose_st.position.x - current_pose.pose.position.x,
                   self.current_goal_pose_st.position.y - current_pose.pose.position.y]
        desired_steering_angle = math.atan2(error_vector[1], error_vector[0])

 
        #desired_steering_angle = np.convolve([desired_steering_angle], np.ones(5) / 5, mode='valid')[0]


        steering_error = -(desired_steering_angle - base_link_angle)

        # Calculate the steering error with angle wrapping
        steering_error = math.atan2(math.sin(steering_error), math.cos(steering_error))

        steering_error_array = np.array([steering_error])
        # Unwrap the angles

        steering_error_array = np.unwrap(steering_error_array)
        # Apply a steering limit to prevent excessive steering angles
        steering_error = max(min(steering_error_array[0], self.steering_error_limit), -self.steering_error_limit)

        steering_error_diff = steering_error - self.steering_last_error
        self.steering_error_sum += steering_error
        steering_output = (
            self.steering_Kp * steering_error +
            self.steering_Ki * self.steering_error_sum +
            self.steering_Kd * steering_error_diff
        )
        steering_error_diff = max(min(steering_error_diff, self.steering_limit), -self.steering_limit)
        self.steering_last_error = steering_error

        return steering_output

    def get_nearest_point(self, plan_msg):
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

        if min_distance <= self.proximity:
            # Reached the nearest point, progress to the next point
            nearest_index = next(
                (i for i, pose in enumerate(plan_msg.poses) if pose.pose == nearest_point), None
            )
            if nearest_index is not None and nearest_index < len(plan_msg.poses) - 1:
                return plan_msg.poses[nearest_index + 0].pose #new

        return nearest_point
    
    def get_nearest_point_st(self, plan_msg):
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

        #if min_distance <= self.proximity_st:
            # Reached the nearest point, progress to the next point
        nearest_index = next(
            (i for i, pose in enumerate(plan_msg.poses) if pose.pose == nearest_point), None
        )
        if nearest_index is not None and nearest_index < len(plan_msg.poses) - 1:
            return plan_msg.poses[nearest_index + 1].pose

        return nearest_point    

    def publish_control_commands(self, steering_control, acceleration_control):
        ackermann_drive_msg = AckermannDrive()
        ackermann_drive_msg.steering_angle = steering_control
        ackermann_drive_msg.acceleration = acceleration_control
        self.cmd_publisher.publish(ackermann_drive_msg)


def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDControllerNode()
    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


