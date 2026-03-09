import controller_tuning.trajectory.profiles as profiles
from controller_tuning.trajectory.utils import Pose5, VelCmd

from enum import StrEnum

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType 
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TwistStamped, Twist, Vector3
from tf_transformations import quaternion_from_euler


# For checking if simulation is running and using correct time
SIM_CLOCK_TOPIC = "/clock"
SIM_TIME_PARAM = "use_sim_time"

# Topics and publishing
REFERENCE_TOPIC = "/reference_trajectory"
COMMAND_TOPIC = "/diff_drive_controller/cmd_vel"
PUBLISH_FREQUENCY = 100  # Hz
FRAME_ID = "odom"  # For Header message
# Defaults for physical constraints
MAX_VELOCITY = 0.5  # m/s
MIN_VELOCITY = -0.5  # m/s
MAX_LINEAR_ACCELERATION = 2.0  # m/s^2
MIN_LINEAR_ACCELERATION = -1.5  # m/s^2
MAX_ANGULAR_VELOCITY = 2.0  # rad/s
MIN_ANGULAR_VELOCITY = -2.0  # rad/s
MAX_ANGULAR_ACCELERATION = 4.0  # rad/s^2
MIN_ANGULAR_ACCELERATION = -4.0  # rad/s^2 
LAT_ACCEL_FACTOR = 0.5  # Multiply with acceleration constraint to conservatively approximate latitudinal constraint
# Defaults for initial pose
INIT_X = 0  # m
INIT_Y = 0  # m
INIT_Z = 0  # m
INIT_YAW = 0  # rads

# Define different QoS profiles based on different needs
REF_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,  # Keep last 10
    durability=DurabilityPolicy.TRANSIENT_LOCAL  # New subscriber receives all stored (last 10) messages
)
CMD_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE  # New subscriber only receives new messages
)

class TrajectoryType(StrEnum):
    SIN = "sinusoid"
    CIRCLE = "circle"
    FIG_EIGHT = "figure-eight"


def symmetric_constraint(
    min_val: float,
    max_val: float
) -> float:
    if min_val < 0:
        return min(abs(min_val), abs(max_val))
    else:
        return max_val


class TrajectoryPublisher(Node):
    """
    ROS 2 node that generates a trajectory profile and periodically publishes poses and velocity commands.
    
    - Parses parameter values to determine topics to publish on, type of trajectory profile to generate,
        and physical constraints and initial pose to use during trajectory generation.
    - Creates a timer using provided controller frequency and creates publishers for pose and velocity 
        commands respectively.
    - Generates the specified trajectory profile.
    - Synchronously publishes each trajectory pose (time + position + yaw) and velocity command (linear x velocity
        and angular yaw velocity) using wall clock time. 
    - Automatically shuts itself down when the full trajectory has been published.

    """
    def __init__(self):
        super().__init__("trajectory_publisher")

        # Mandatory
        self.declare_parameter(
            name="type",
            value="",  # Signals that no trajectory type was provided
            descriptor=ParameterDescriptor(
                description="The type of trajectory to generate for the reference: '{}', '{}', or '{}'".format(
                        TrajectoryType.SIN, TrajectoryType.CIRCLE, TrajectoryType.FIG_EIGHT
                ),
                type=ParameterType.PARAMETER_STRING
            )
        )
        # Check if value was provided
        if not self.get_parameter("type").value:
            raise RuntimeError(
                "Parameter 'type' must be set to either '{}', '{}', or '{}'"
                .format(TrajectoryType.SIN, TrajectoryType.CIRCLE, TrajectoryType.FIG_EIGHT)
            )

        # Auto-handle sim time parameter
        if self.get_parameter(SIM_TIME_PARAM).value == False:
            if self.count_publishers(SIM_CLOCK_TOPIC) > 0:
                # Assume simulation is running and we need to use corresponding time
                self.get_logger().info(
                    "Detected '{}' topic that indicates simulation is running. Setting '{}' parameter to 'true'".format(
                        SIM_CLOCK_TOPIC,
                        SIM_TIME_PARAM
                    )
                )
                self.set_parameters([
                    Parameter(
                        SIM_TIME_PARAM,
                        Parameter.Type.BOOL,
                        True
                    )
                ])
            
        
        self.declare_parameter(
            name="reference_topic",
            value=REFERENCE_TOPIC,
            descriptor=ParameterDescriptor(
                description="The topic to publish reference trajectory PoseStamped on",
                type=ParameterType.PARAMETER_STRING
            )
        )
        self.declare_parameter(
            name="command_topic",
            value=COMMAND_TOPIC,
            descriptor=ParameterDescriptor(
                description="The topic to publish trajectory TwistStamped commands on",
                type=ParameterType.PARAMETER_STRING
            )
        )
        self.declare_parameter(
            name="publish_frequency",
            value=PUBLISH_FREQUENCY,
            descriptor=ParameterDescriptor(
                description=("The frequency (Hz) at which to publish poses and velocity commands. Used to"
                " determine how many times to sample a given trajectory. Should be set at a value"
                " significantly lower than controller frequency"
                ),
                type=ParameterType.PARAMETER_DOUBLE
            )
        )
        self.declare_parameter(
            name="max_vel",
            value=MAX_VELOCITY,
            descriptor=ParameterDescriptor(
                description=(
                    "Constraint on maximum value of linear velocity. Used in tandem with minimum value"
                    " to determine how to constrain linear velocities in trajectories"
                ),
                type=ParameterType.PARAMETER_DOUBLE
            )
        )
        self.declare_parameter(
            name="min_vel",
            value=MIN_VELOCITY,
            descriptor=ParameterDescriptor(
                description=(
                    "Constraint on minimum value of linear velocity. Used in tandem with maximum value"
                    " to determine how to constrain linear velocities in trajectories"
                ),
                type=ParameterType.PARAMETER_DOUBLE
            )
        )
        self.declare_parameter(
            name="max_ang_vel",
            value=MAX_ANGULAR_VELOCITY,
            descriptor=ParameterDescriptor(
                description=(
                    "Constraint on maximum value of angular velocity. Used in tandem with minimum"
                    " to determine how to constrain angular velocities in trajectories"
                ),
                type=ParameterType.PARAMETER_DOUBLE
            )
        )
        self.declare_parameter(
            name="min_ang_vel",
            value=MIN_ANGULAR_VELOCITY,
            descriptor=ParameterDescriptor(
                description=(
                    "Constraint on minimum value of angular velocity. Used in tandem with maximum value"
                    " to determine how to constrain angular velocities in trajectories"
                ),
                type=ParameterType.PARAMETER_DOUBLE
            )
        )
        self.declare_parameter(
            name="max_lin_accel",
            value=MAX_LINEAR_ACCELERATION,
            descriptor=ParameterDescriptor(
                description=(
                    "Constraint on maximum value of linear acceleration. Used in tandem with minimum value"
                    " to determine how to constrain linear accelerations in trajectories"
                ),
                type=ParameterType.PARAMETER_DOUBLE
            )
        )
        self.declare_parameter(
            name="min_lin_accel",
            value=MIN_LINEAR_ACCELERATION,
            descriptor=ParameterDescriptor(
                description=(
                    "Constraint on minimum value of linear acceleration. Used in tandem with maximum value"
                    " to determine how to constrain linear accelerations in trajectories"
                ),
                type=ParameterType.PARAMETER_DOUBLE
            )
        )
        self.declare_parameter(
            name="max_ang_accel",
            value=MAX_ANGULAR_ACCELERATION,
            descriptor=ParameterDescriptor(
                description=(
                    "Constraint on maximum value of angular acceleration. Used in tandem with minimum value"
                    " to determine how to constrain angular accelerations in trajectories"
                ),
                type=ParameterType.PARAMETER_DOUBLE
            )
        )
        self.declare_parameter(
            name="min_ang_accel",
            value=MIN_ANGULAR_ACCELERATION,
            descriptor=ParameterDescriptor(
                description=(
                    "Constraint on minimum value of angular acceleration. Used in tandem with maximum value"
                    " to determine how to constrain angular accelerations in trajectories"
                ),
                type=ParameterType.PARAMETER_DOUBLE
            )
        )
        self.declare_parameter(
            name="lat_accel_factor",
            value=LAT_ACCEL_FACTOR,
            descriptor=ParameterDescriptor(
                description=(
                    "Factor to multiply linear (longitudinal) acceleration constraint by to determine"
                    " how to constrain lateral accelerations in trajectories"
                ),
                type=ParameterType.PARAMETER_DOUBLE
            )
        )
        self.declare_parameter(
            name="init_x",
            value=INIT_X,
            descriptor=ParameterDescriptor(
                description="Initial x position (m)",
                type=ParameterType.PARAMETER_DOUBLE
            )
        )
        self.declare_parameter(
            name="init_y",
            value=INIT_Y,
            descriptor=ParameterDescriptor(
                description="Initial y position (m)",
                type=ParameterType.PARAMETER_DOUBLE
            )
        )
        self.declare_parameter(
            name="init_z",
            value=INIT_Z,
            descriptor=ParameterDescriptor(
                description="Initial z position (m)",
                type=ParameterType.PARAMETER_DOUBLE
            )
        )
        self.declare_parameter(
            name="init_yaw",
            value=INIT_YAW,
            descriptor=ParameterDescriptor(
                description="Initial yaw angle (rads)",
                type=ParameterType.PARAMETER_DOUBLE
            )
        )

        # Reference trajectory for logging
        self.ref_trajectory_publisher = self.create_publisher(
            msg_type=PoseStamped,
            topic=self.get_parameter(name="reference_topic").value,
            qos_profile=REF_QOS
        )
        # Velocity commands for controller
        self.velocity_cmd_publisher = self.create_publisher(
            msg_type=TwistStamped,
            topic=self.get_parameter(name="command_topic").value,
            qos_profile=CMD_QOS
        )

        trajectory_type = self.get_parameter("type").value
        max_vel = self.get_parameter("max_vel").value
        min_vel = self.get_parameter("min_vel").value
        max_lin_accel = self.get_parameter("max_lin_accel").value
        min_lin_accel = self.get_parameter("min_lin_accel").value
        max_ang_vel = self.get_parameter("max_ang_vel").value
        min_ang_vel = self.get_parameter("min_ang_vel").value
        max_ang_accel = self.get_parameter("max_ang_accel").value
        min_ang_accel = self.get_parameter("min_ang_accel").value        
        # Check constraints
        # Velocity
        if min_vel >= max_vel:
            raise ValueError("min_vel must be < max_vel")
        elif min_vel >= 0:
            raise ValueError("min_vel must be negative")
        elif max_vel <= 0:
            raise ValueError("max_vel must be positive")
        # Linear acceleration
        if min_lin_accel >= max_lin_accel:
            raise ValueError("min_lin_accel must be < max_lin_accel")
        elif min_lin_accel >= 0:
            raise ValueError("min_lin_accel must be negative")
        elif max_lin_accel <= 0:
            raise ValueError("max_lin_accel must be positive")
        # Angular velocity
        if min_ang_vel >= max_ang_vel:
            raise ValueError("min_ang_vel must be < max_ang_vel")
        elif min_ang_vel >= 0:
            raise ValueError("min_ang_vel must be negative")
        elif max_ang_vel <= 0:
            raise ValueError("max_ang_vel must be positive")
        # Angular acceleration
        if min_ang_accel >= max_ang_accel:
            raise ValueError("min_ang_accel must be < max_ang_accel")
        elif min_ang_accel >= 0:
            raise ValueError("min_ang_accel must be negative")
        elif max_ang_accel <= 0:
            raise ValueError("max_ang_accel must be positive")
        
        # Get minimum of absolute values of max/min constraints to avoid exceeding either during trajectories
        vel_constraint = symmetric_constraint(min_vel, max_vel)
        lin_accel_constraint = symmetric_constraint(min_lin_accel, max_lin_accel)
        ang_vel_constraint = symmetric_constraint(min_ang_vel, max_ang_vel)
        ang_accel_constraint = symmetric_constraint(min_ang_accel, max_ang_accel)
        lat_accel_constraint = self.get_parameter("lat_accel_factor").value * lin_accel_constraint 

        init_pose = Pose5(
            t=0,
            x=self.get_parameter("init_x").value,
            y=self.get_parameter("init_y").value,
            z=self.get_parameter("init_z").value,
            yaw=self.get_parameter("init_yaw").value
        )
        
        self.get_logger().info("Generating {} trajectory profile...".format(trajectory_type))
        if trajectory_type == TrajectoryType.SIN:
            self.trajectory = profiles.sin_velocity_line(
                sample_frequency=self.get_parameter("publish_frequency").value,
                max_long_vel=max_vel,
                max_long_accel=max_lin_accel,
                init_pose=init_pose
            )
        elif trajectory_type == TrajectoryType.CIRCLE:
            circle_profile = profiles.circle(
                sample_frequency=self.get_parameter("publish_frequency").value,
                max_angular_vel=ang_vel_constraint,
                max_lat_accel=lat_accel_constraint,  # Approximate conservatively
                init_pose=init_pose
            )
            # Add acceleration ramps at start and end to achieve dynamic feasability 
            self.trajectory = profiles.add_accel_ramps(
                trajectory=circle_profile,
                max_long_accel=lin_accel_constraint,
                max_ang_accel=ang_accel_constraint,
                sample_frequency=self.get_parameter("publish_frequency").value,
                init_pose=init_pose
            )
        elif trajectory_type == TrajectoryType.FIG_EIGHT:
            fig_eight_profile = profiles.figure_eight(
                sample_frequency=self.get_parameter("publish_frequency").value,
                max_long_vel=vel_constraint,
                max_angular_vel=ang_vel_constraint,
                max_long_accel=lin_accel_constraint,
                init_pose=init_pose
            )
            self.trajectory = profiles.add_accel_ramps(
                trajectory=fig_eight_profile,
                max_long_accel=lin_accel_constraint,
                max_ang_accel=ang_accel_constraint,
                sample_frequency=self.get_parameter("publish_frequency").value,
                init_pose=init_pose
            )
        else:
            raise ValueError(
                "{} is an unrecognised trajectory type. Recognised types are: ('{}', '{}', '{}')".format(
                    trajectory_type,
                    TrajectoryType.SIN,
                    TrajectoryType.CIRCLE,
                    TrajectoryType.FIG_EIGHT
                )
            )
        
        # Check trajectory 
        num_poses = len(self.trajectory.poses)
        num_vels = len(self.trajectory.vels)
        if num_poses == 0:
            raise RuntimeError("Number of Pose5 objects in generated trajectory profile is zero. Something has gone wrong...") 
        elif num_vels == 0:
            raise RuntimeError("Number of VelCmd objects in generated trajectory profile is zero. Something has gone wrong...")
        elif num_poses != num_vels:
            raise RuntimeError(
                "Number of Pose5 objects ({}) is not equal to number of VelCmd objects ({}). Something has gone wrong...".format(num_poses, num_vels)
            )
        
        self.num_samples = num_poses
        self.idx = 0 

        self.get_logger().info(
            "Generated {} trajectory with sampling frequency = {}Hz, initial point (t, x, y, z, yaw) = ({}, {}, {}, {}, {}), and number of samples = {}"
            .format(
                trajectory_type,
                self.get_parameter("publish_frequency").value,
                init_pose.t,
                init_pose.x,
                init_pose.y,
                init_pose.z,
                init_pose.yaw,
                self.num_samples
            )
        )
        
        # Create timer to sync publishing of trajectory points and velocity commands at specific frequency
        timer_period = 1 / self.get_parameter("publish_frequency").value
        self.timer = self.create_timer(
            timer_period_sec=timer_period,
            callback=self.timer_callback
        )

        # Define number of warmup cycles to run before publishing trajectory
        self.warmup_cycles = int(2 * PUBLISH_FREQUENCY)
        self.warmup_idx = 0

        self.get_logger().info("Publishing warmup...")

    def timer_callback(self) -> None:
        """
        Periodically and sequentially publish trajectory pose and velocity command
        to respective topics.

        """
        # Create Header for pose and velocity command 
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = FRAME_ID

        # Publish zero velocity during warmup to provide time for controller/Gazebo buffers to clear, DDS connections to establish, etc...
        if self.warmup_idx < self.warmup_cycles:
            vel_cmd = VelCmd(linear_x=0.0, angular_z=0.0)
            self.publish_vel_cmd(vel_cmd, header)
            self.warmup_idx += 1

            if self.warmup_idx == self.warmup_cycles:
                # Warmup completed
                self.get_logger().info("Publishing trajectory...")

            return

        if self.idx < self.num_samples:
            # There are still points and velocities left to publish
            # Get current trajectory point and velocity command
            pose = self.trajectory.poses[self.idx]
            if self.idx < self.num_samples - 1:
                vel_cmd = self.trajectory.vels[self.idx]
            else:
                # Final point in trajectory - command robot to stop because we only need 
                # the pose now for the reference
                vel_cmd = VelCmd(linear_x=0.0, angular_z=0.0)

            self.publish_pose(pose, header)
            self.publish_vel_cmd(vel_cmd, header)

            self.idx += 1
        else:
            # Full trajectory has been published - shut down node
            self.timer.cancel()
            self.get_logger().info("Full trajectory has been published. Shutting down...")
            self.destroy_node()
            rclpy.shutdown()
            

    def publish_pose(
        self,
        pose: Pose5,
        header: Header
    ) -> None:
        """
        Create PoseStamped message using trajectory pose data and publish to configured topic.

        Args:
            pose (Pose5): Object storing time, position, and yaw data from a trajectory element.
            header (Header): Message object storing frame ID and time stamp.

        """
        # Construct PoseStamped message
        pose_stamped_msg = PoseStamped()
        pose_msg = Pose()
        # Set (x, y, z) position
        pose_msg.position = Point(
            x=float(pose.x),
            y=float(pose.y),
            z=float(pose.z)
        )
        # Convert Euler to Quaternion and set orientation
        quaternion = quaternion_from_euler(0, 0, pose.yaw, "rzyx")  # Intrinsic ZYX Euler sequence used for consistency with ROS 2
        pose_msg.orientation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )
        pose_stamped_msg.pose = pose_msg
        
        # Add the Header
        pose_stamped_msg.header = header

        self.ref_trajectory_publisher.publish(pose_stamped_msg)

    def publish_vel_cmd(
        self,
        vel_cmd: VelCmd,
        header: Header
    ) -> None:
        """
        Create TwistStamped message using trajectory velocity command data and publish to configured topic.

        Args:
            vel_cmd (VelCmd): Object storing linear and angular velocity commands from a trajectory element.
            header (Header): Message object storing frame ID and time stamp.

        """
        # Construct TwistStamped message for use by differential drive controller
        twist_stamped_msg = TwistStamped()
        # First construct Twist message
        twist_msg = Twist()
        twist_msg.linear = Vector3(
            x=float(vel_cmd.linear_x),
            y=0.0,
            z=0.0
        )
        twist_msg.angular = Vector3(
            x=0.0,
            y=0.0,
            z=float(vel_cmd.angular_z)
        )
        twist_stamped_msg.twist = twist_msg

        # Add the Header
        twist_stamped_msg.header = header

        self.velocity_cmd_publisher.publish(twist_stamped_msg)


def main(args=None):
    """
    Entry point for TrajectoryPublisher node.

    """
    # Shutdown is automatically triggered when full trajectory has been published
    rclpy.init(args=args)
    rclpy.spin(TrajectoryPublisher())


if __name__ == "__main__":
    main()
