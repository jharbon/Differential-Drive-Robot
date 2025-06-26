import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType 
from rclpy.qos import qos_profile_default
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist, Vector3
from tf_transformations import quaternion_from_euler
from controller_tuning.trajectory_generator import sinusoid_velocity_profile, circle_velocity_profile, TrajectoryGenerator
from controller_tuning.trajectory_utils import Pose5, VelCmd

# Defaults for topics and publishing
REFERENCE_TOPIC = "/reference_trajectory"
REFERENCE_TYPE = "sinusoid"
COMMAND_TOPIC = "/cmd_vel"
PUBLISH_FREQUENCY = 10  # Hz
POSE_FRAME_ID = "odom"

# Defaults for general trajectory constraints
TIME_DELTA = 1 / PUBLISH_FREQUENCY  # s
MAX_VELOCITY = 0.5  # m/s
MIN_VELOCITY = -0.5  # m/s
MAX_ANGULAR_VELOCITY = 2.0  # rad/s
N_POINTS = 100  # Number of points defining trajectory

# Defaults for initial position and orientation
INIT_X = 0  # m
INIT_Y = 0  # m
INIT_Z = 0  # m
INIT_YAW = 0  # rad 


class TrajectoryPublisher(Node):
    """
    ROS 2 node which generates a trajectory using the TrajectoryGenerator class, uses a timer to
    synchronously publish the reference pose (time + position + yaw) and velocity commands to 
    respective topics, and shuts itself down when the full trajectory has been published.

    """
    def __init__(self):
        super().__init__("trajectory_publisher")

        # Topics
        self.declare_parameter(
            name="reference_topic",
            descriptor=ParameterDescriptor(
                description="The topic to publish reference trajectory PoseStamped on",
                type=ParameterType.PARAMETER_STRING
            ),
            value=REFERENCE_TOPIC
        )
        self.declare_parameter(
            name="command_topic",
            descriptor=ParameterDescriptor(
                description="The topic to publish reference trajectory Twist commands on",
                type=ParameterType.PARAMETER_STRING
            ),
            value=COMMAND_TOPIC
        )
        self.declare_parameter(
            name="publish_frequency",
            descriptor=ParameterDescriptor(
                description="The frequency (Hz) at which to publish reference trajectory points and velocity commands",
                type=ParameterType.PARAMETER_DOUBLE
            ),
            value=PUBLISH_FREQUENCY
        )
        self.declare_parameter(
            name="reference_type",
            descriptor=ParameterDescriptor(
                description="The type of trajectory ('sinusoid' or 'circle') to generate for the reference",
                type=ParameterType.PARAMETER_STRING
            ),
            value=REFERENCE_TYPE
        )
        self.declare_parameter(
            name="max_velocity",
            descriptor=ParameterDescriptor(
                description="The maximum size of linear velocity commands for each point on the trajectory",
                type=ParameterType.PARAMETER_DOUBLE
            ),
            value=MAX_VELOCITY
        )
        self.declare_parameter(
            name="min_velocity",
            descriptor=ParameterDescriptor(
                description="The minimum size of linear velocity commands for each point on the trajectory",
                type=ParameterType.PARAMETER_DOUBLE
            ),
            value=MIN_VELOCITY
        )
        self.declare_parameter(
            name="max_angular_velocity",
            descriptor=ParameterDescriptor(
                description="The maximum size of angular velocity commands for each point on the trajectory",
                type=ParameterType.PARAMETER_DOUBLE
            ),
            value=MAX_ANGULAR_VELOCITY
        )
        self.declare_parameter(
            name="n_points",
            descriptor=ParameterDescriptor(
                description="The number of points used to define the trajectory",
                type=ParameterType.PARAMETER_INTEGER
            ),
            value=N_POINTS
        )

        if self.get_parameter(name="n_points").value < 2:
            raise ValueError("Number of points must be >= 2")
        
        # Reference trajectory for logging
        self.ref_trajectory_publisher = self.create_publisher(
            msg_type=PoseStamped,
            topic=self.get_parameter(name="reference_topic").value,
            qos_profile=qos_profile_default
        )
        # Velocity commands for controller
        self.velocity_cmd_publisher = self.create_publisher(
            msg_type=Twist,
            topic=self.get_parameter(name="command_topic").value,
            qos_profile=qos_profile_default
        )

        # Create timer to sync publishing of trajectory points and velocity commands at specific frequency
        timer_period = 1 / self.get_parameter("publish_frequency").value
        self.timer = self.create_timer(
            timer_period_sec=timer_period,
            callback=self.timer_callback
        )

        # Generate velocity profile
        reference_type = self.get_parameter("reference_type").value
        n_vels = self.get_parameter("n_points").value - 1
        max_velocity = self.get_parameter("max_velocity").value
        min_velocity = self.get_parameter("min_velocity").value
        max_angular_velocity = self.get_parameter("max_angular_velocity").value
        self.get_logger().info("Generating {} velocity profile...".format(reference_type))
        if reference_type == "sinusoid":
            v_profile = sinusoid_velocity_profile(
                n=n_vels,
                max_v=max_velocity,
                min_v=min_velocity
            )
            self.get_logger().info(
                "Generated {} velocity profile with number of velocities = {}, max velocity = {}, and min velocity = {}"
                .format(
                    reference_type,
                    len(v_profile),
                    max_velocity,
                    min_velocity
                )
            )
        elif reference_type == "circle":
            v_profile = circle_velocity_profile(
                n=n_vels,
                linear_v=max_velocity,
                angular_v=max_angular_velocity
            )
            self.get_logger().info(
                "Generated {} velocity profile with number of velocities = {}, linear velocity = {}, and angular velocity = {}"
                .format(
                    reference_type,
                    len(v_profile),
                    max_velocity,
                    max_angular_velocity
                )
            )
        else:
            raise ValueError("{} is an unrecognised reference trajectory type. Recognised types are: ('sinusoid', 'circle')".format(reference_type))

        init_pose = Pose5(
            t=0,
            x=INIT_X,
            y=INIT_Y,
            z=INIT_Z,
            yaw=INIT_YAW
        )
        t_delta = 1 / self.get_parameter("publish_frequency").value
        self.get_logger().info("Generating trajectory...")
        trajectory_generator = TrajectoryGenerator(
            t_delta=t_delta,
            init_pose=init_pose,
            v_profile=v_profile
        )
        self.trajectory = trajectory_generator.generate_trajectory()
        self.get_logger().info(
            "Generated {} trajectory with time delta = {}, initial point (t, x, y, z, yaw) = ({}, {}, {}, {}, {}), and number of points = {}"
            .format(
                reference_type,
                t_delta,
                init_pose.t,
                init_pose.x,
                init_pose.y,
                init_pose.z,
                init_pose.yaw,
                len(self.trajectory)
            )
        )
        self.idx_trajectory = 0   
        self.get_logger().info("Publishing trajectory...")

    def timer_callback(self) -> None:
        """
        Periodically and sequentially publish trajectory pose and velocity command
        to respective topics.

        """
        if self.idx_trajectory < self.get_parameter("n_points").value:
            # There are still points and velocities left to publish
            # Get current trajectory point and velocity command
            trajectory_element = self.trajectory[self.idx_trajectory]
            trajectory_pose = trajectory_element[0]
            trajectory_vel_cmd = trajectory_element[1]

            self.publish_trajectory_pose(trajectory_pose=trajectory_pose)
            self.publish_trajectory_vel_cmd(trajectory_vel_cmd=trajectory_vel_cmd)

            self.idx_trajectory += 1
        else:
            # Full trajectory has been published - shut down node
            self.get_logger().info("Full trajectory has been published. Node shutting down...")
            rclpy.shutdown()

    def publish_trajectory_pose(self, trajectory_pose: Pose5) -> None:
        """
        Create PoseStamped message using trajectory pose data and publish to configured topic.

        Args:
            trajectory_pose (Pose5): Object storing time, position, and yaw data from a trajectory element.

        """
        # Construct PoseStamped message
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        pose_stamped_msg.header.frame_id = POSE_FRAME_ID
        pose_msg = Pose()
        # Set (x, y, z) position
        pose_msg.position = Point(
            x=float(trajectory_pose.x),
            y=float(trajectory_pose.y),
            z=float(trajectory_pose.z)
        )
        # Convert Euler to Quaternion and set orientation
        quaternion = quaternion_from_euler(0, 0, trajectory_pose.yaw, "rzyx")  # Intrinsic ZYX Euler sequence used for consistency with ROS 2
        pose_msg.orientation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )
        pose_stamped_msg.pose = pose_msg

        self.ref_trajectory_publisher.publish(pose_stamped_msg)

    def publish_trajectory_vel_cmd(self, trajectory_vel_cmd: VelCmd) -> None:
        """
        Create Twist message using trajectory velocity command data and publish to configured topic.

        Args:
            trajectory_vel_cmd (VelCmd): Object storing linear and angular velocity commands from a trajectory element.

        """
        # Construct Twist message for use by differential drive controller
        twist_msg = Twist()
        twist_msg.linear = Vector3(
            x=float(trajectory_vel_cmd.linear_x),
            y=0.0,
            z=0.0
        )
        twist_msg.angular = Vector3(
            x=0.0,
            y=0.0,
            z=float(trajectory_vel_cmd.angular_z)
        )

        self.velocity_cmd_publisher.publish(twist_msg)


def main(args=None):
    """
    Entry point for TrajectoryPublisher node.

    """
    rclpy.init(args=args)
    rclpy.spin(TrajectoryPublisher())


if __name__ == "__main__":
    main()
