import rclpy
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry

class JetracerCrtl:
    def init(self, webots_node, properties):
        # Declare the robot name and fix the timestep
        self.robot = webots_node.robot
        self.timestep = int(self.robot.getBasicTimeStep())
        self.robot_name = self.robot.getName()
       # Init Camera
        camera = self.robot.getDevice("camera")
        camera.enable(self.timestep)
        # camera.recognitionEnable(self.timestep)

        # Init Lidar
        lidar = self.robot.getDevice("lidar")
        lidar.enable(self.timestep)



        ## Initialize imu
        self.imu = self.robot.getDevice("inertial_unit")
        self.imu.enable(self.timestep)


        # Initialize webots driver node
        rclpy.init(args=None)
        self.namespace = str(self.robot_name)
        self.jetracer_driver = rclpy.create_node(
                            'jetracer_driver',
                            namespace=self.namespace,
                            allow_undeclared_parameters=True,
                            automatically_declare_parameters_from_overrides=True)

        # log basic informations
        self.jetracer_driver.get_logger().info('robot name:\t {}'.format(self.robot_name))
        self.jetracer_driver.get_logger().info('basic timestep:\t {}'.format(self.timestep))
        self.jetracer_driver.get_logger().info('properties:\t {}'.format(properties))

        # Declare Subscriptions
        msg_type = AckermannDrive()
        self.target_cmd = msg_type
        self.jetracer_driver.create_subscription(msg_type, '/{}/cmd_ackermann'.format(self.namespace), self.setpoint_callback, 1)
        self.stop_subscription = self.jetracer_driver.create_subscription(Empty, '/stop', self.stop, 10)
        
        # Initialize flags
        self.initialization = True
        self.emergency_stop = False

        self.jetracer_driver.get_logger().info("jetson driver {} started".format(self.namespace))

    def init_setpoint(self):
        self.target_cmd.acceleration = 0.0
        self.target_cmd.steering_angle = 0.0

    def setpoint_callback(self, ackermann):
        """
        float32 steering_angle
        float32 steering_angle_velocity
        float32 speed
        float32 acceleration
        float32 jerk
        """
        print('Received setpoint.')
        self.target_cmd = ackermann

    def stop(self,_):
        self.emergency_stop = True

    def send_motor_cmd(self, cmd_throttle, cmd_steering_angle):
        self.robot.setCruisingSpeed(cmd_throttle)
        self.robot.setSteeringAngle(cmd_steering_angle)

    def step(self):

        rclpy.spin_once(self.jetracer_driver, timeout_sec=0)
        # self.time = self.robot.getTime()
        
        if self.initialization:
            self.initialization = False
            self.init_setpoint()
            
        cmd_throttle        =  float(self.target_cmd.acceleration)     # [-1,1]
        cmd_steering_angle  =  float(self.target_cmd.steering_angle)   # [-1,1]

        if self.emergency_stop:
            cmd_throttle        =  0.0  # [-1,1]
            cmd_steering_angle  =  0.0  # [-1,1]


        string = "Throttle = {}\tSteering = {}".format(cmd_throttle, cmd_steering_angle)
        # print(string)
        
        self.send_motor_cmd(cmd_throttle, cmd_steering_angle)


