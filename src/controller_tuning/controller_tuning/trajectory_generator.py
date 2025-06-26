from controller_tuning.trajectory_utils import Pose5, VelCmd  
from typing import List, Tuple
from math import sin, cos, atan2, pi


def sinusoid_velocity_profile(
        n: int,
        max_v: float,
        min_v: float
) -> List[VelCmd]:
    """
    Generate velocities with a magnitude scaled by a sinusoid function and a constant direction.  

    Args:
        n (int): Number of velocities to generate.
        max_v (float): Maximum linear velocity which can be generated.
        min_v (float): Minimum linear velocity which can be generated.

    Returns:
        List[VelCmd]: Sequence of velocity commands which collectively produce a 1D A->B->A path when integrated.

    """
    rad_delta = 2 * pi / n
    angular_z = 0  # Straight line path
    vel_cmds = []
    for i in range(n):
        rads = i * rad_delta
        linear_x = max_v * sin(rads)
        if linear_x < min_v:
            # Enforce min velocity even if it warps the shape of the trough
            linear_x = min_v

        vel_cmds.append(VelCmd(linear_x=linear_x, angular_z=angular_z))

    return vel_cmds


def circle_velocity_profile(
        n: int,
        linear_v: float,
        angular_v: float
) -> List[VelCmd]:
    """
    Generates velocities with a constant magnitude and a rotating direction.
     
    Args:
        n (int): Number of velocities to generate.
        linear_v (float): Constant linear velocity magnitude.
        angular_v (float): Constant angular velocity magnitude.

    Returns:
        List[VelCmd]: Sequence of velocity commands which collectively produce a 2D circular path when integrated.

    """ 
    return [VelCmd(linear_x=linear_v, angular_z=angular_v)] * n 


class TrajectoryGenerator:
    """
    Generates a list of [Pose5, VelCmd] elements which can be used both as a sequence of velocity commands and
    as a reference for what the resulting path should look like.

    Attributes:
        t_delta (float): Time difference between each trajectory element.
        init_pose (Pose5): Initial pose (defined here as time + position + yaw).
        v_profile (List[VelCmd]): Sequence of velocity commands to be integrated.

    """
    def __init__(
            self,
            t_delta: float,  # s
            init_pose: Pose5,  # s,m,m,m,rad   
            v_profile: List[VelCmd]  # m/s,rad/s
    ):
        self.t_delta = t_delta
        self.init_pose = init_pose
        self.v_profile = v_profile

    def generate_pose(
            self,
            prev_pose: Pose5,
            vel_cmd: VelCmd     
    ) -> Pose5:
        """
        Generate a new pose.

        Args:
            prev_pose (Pose5): Previous pose with components to be added to time step and integrated velocites.
            vel_cmd (VelCmd): Velocity command with components for integration.

        Returns:
            Pose5: New absolute time, position, and yaw.

        """
        t = prev_pose.t + self.t_delta
        # Integrate velocities to update position and orientation
        mid_yaw = prev_pose.yaw + vel_cmd.angular_z * self.t_delta / 2  # Use yaw midpoint to improve accuracy of x and y integration
        x = prev_pose.x + vel_cmd.linear_x * cos(mid_yaw) * self.t_delta
        y = prev_pose.y + vel_cmd.linear_x * sin(mid_yaw) * self.t_delta
        z = prev_pose.z
        yaw = prev_pose.yaw + vel_cmd.angular_z * self.t_delta
        # Wrap yaw to keep it in [-pi, pi] range
        yaw = atan2(sin(yaw), cos(yaw))

        return Pose5(t, x, y, z, yaw)
    
    def generate_trajectory(self) -> List[Tuple[Pose5, VelCmd]]:
        """
        Generate the full trajectory corresponding to the time delta, initial pose, and velocity profile.

        Returns:
            List[Tuple[Pose5, VelCmd]]: List of trajectory elements where each element is a tuple containing
            the pose and the velocity command which generates the following pose.

        """
        trajectory = []
        pose = self.init_pose
        for cmd in self.v_profile:
            trajectory.append(
                (pose, cmd)
            )
            # Use current velocity command to generate the next pose
            pose = self.generate_pose(
                prev_pose=pose,
                vel_cmd=cmd
            )
        # Add a "null"/stationary VelCmd as a placeholder in the final element
        trajectory.append(
            (pose, VelCmd(linear_x=0, angular_z=0))
        )

        return trajectory
