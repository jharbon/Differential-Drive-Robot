from controller_tuning.trajectory.utils import Pose5, VelCmd, TrajectoryProfile

from math import ceil, sin, cos, atan2, pi, sqrt

# Constants for figure-eight curve
FIGURE_EIGHT_HALF_WIDTH = 0.25  # m
FIGURE_EIGHT_HALF_HEIGHT = 0.25  # m
FIGURE_EIGHT_INITIAL_W = 1.0


def compute_ramp_duration(
    lin_v_delta: float,
    long_accel: float,
    ang_v_delta: float,
    ang_accel: float
) -> float:
    lin_duration = abs(lin_v_delta / long_accel)
    ang_duration = abs(ang_v_delta / ang_accel)
    # Get largest of the two required ramp durations
    return max(lin_duration, ang_duration)

def build_ramp_profile(
    init_pose: Pose5,
    init_lin_v: float,
    target_lin_v: float,
    init_ang_v: float,
    target_ang_v: float,
    max_long_accel: float,
    max_ang_accel: float,
    duration: float,
    time_delta: float
) -> tuple[list[Pose5], list[VelCmd], list[float]]:
    poses = []
    vels = []
    accels = []
    x = init_pose.x
    y = init_pose.y
    yaw = init_pose.yaw
    lin_v_delta = target_lin_v - init_lin_v
    ang_v_delta = target_ang_v - init_ang_v
    long_accel = max_long_accel if lin_v_delta >= 0 else -(max_long_accel)
    ang_accel = max_ang_accel if ang_v_delta >= 0 else -(max_ang_accel)
    num_steps = ceil(duration / time_delta)
    for i in range(num_steps):
        lin_v = init_lin_v + long_accel * i * time_delta
        # Computed ramp durations for linear velocity and angular velocity will in general be
        # unequal; account for this with clamp
        if (lin_v_delta > 0 and lin_v > target_lin_v) or (lin_v_delta < 0 and lin_v < target_lin_v):
            lin_v = target_lin_v

        ang_v = init_ang_v + ang_accel * i * time_delta 
        # Repeat clamp
        if (ang_v_delta > 0 and ang_v > target_ang_v) or (ang_v_delta < 0 and ang_v < target_ang_v):
            ang_v = target_ang_v

        yaw_delta = ang_v * time_delta
        # Compute mid-yaw for more accurate velocity integration for x and y
        mid_yaw = yaw + (yaw_delta / 2)
        yaw += yaw_delta

        x += lin_v * cos(mid_yaw) * time_delta
        y += lin_v * sin(mid_yaw) * time_delta

        poses.append(Pose5(
            t=(init_pose.t + i * time_delta),
            x=x,
            y=y,
            z=init_pose.z,
            yaw=yaw
        ))

        vels.append(VelCmd(
            linear_x=lin_v,
            angular_z=ang_v
        ))

        if lin_v == target_lin_v:
            accels.append(0.0)
        else:
            accels.append(long_accel)

    return (poses, vels, accels)

def add_accel_ramps(
    trajectory: TrajectoryProfile,
    max_long_accel: float,
    max_ang_accel: float,
    sample_frequency: int,
    init_pose: Pose5 
) -> TrajectoryProfile:
    time_delta = 1 / sample_frequency
    
    start_duration = compute_ramp_duration(
        lin_v_delta=trajectory.vels[0].linear_x,
        long_accel=max_long_accel,
        ang_v_delta=trajectory.vels[0].angular_z,
        ang_accel=max_ang_accel
    )
    start_poses, start_vels, start_accels = build_ramp_profile(
        init_pose=init_pose,
        init_lin_v=0,
        target_lin_v=trajectory.vels[0].linear_x,
        init_ang_v=0,
        target_ang_v=trajectory.vels[0].angular_z,
        max_long_accel=max_long_accel,
        max_ang_accel=max_ang_accel,
        duration=start_duration,
        time_delta=time_delta
    )

    # Use final start ramp pose to shift all relevant pose values in trajectory and get correct absolute values
    start_final_pose = start_poses[-1]
    shifted_trajectory_poses = []
    for pose in trajectory.poses:
        # Compute ramp offset where init pose is subtracted to avoid over-shifting trajectory which already adds init pose
        ramp_offset_x = start_final_pose.x - init_pose.x
        ramp_offset_y = start_final_pose.y - init_pose.y
        ramp_offset_z = start_final_pose.z - init_pose.z
        ramp_offset_yaw = start_final_pose.yaw - init_pose.yaw

        shifted_trajectory_poses.append(Pose5(
            t=(start_final_pose.t + pose.t),
            x=(ramp_offset_x + pose.x),
            y=(ramp_offset_y + pose.y),
            z=(ramp_offset_z + pose.z),
            yaw=(ramp_offset_yaw + pose.yaw)
        ))       

    end_duration = compute_ramp_duration(
        lin_v_delta=trajectory.vels[-1].linear_x,
        long_accel=max_long_accel,
        ang_v_delta=trajectory.vels[-1].angular_z,
        ang_accel=max_ang_accel
    )
    end_poses, end_vels, end_accels = build_ramp_profile(
        init_pose=shifted_trajectory_poses[-1],
        init_lin_v=trajectory.vels[-1].linear_x,
        target_lin_v=0,
        init_ang_v=trajectory.vels[-1].angular_z,
        target_ang_v=0,
        max_long_accel=max_long_accel,
        max_ang_accel=max_ang_accel,
        duration=end_duration,
        time_delta=time_delta
    )

    return TrajectoryProfile(
        poses=tuple(start_poses + shifted_trajectory_poses + end_poses),
        vels=tuple(start_vels + list(trajectory.vels) + end_vels),
        long_accels=(start_accels + list(trajectory.long_accels) + end_accels) if trajectory.long_accels else ()
    )


def sin_velocity_line(
    sample_frequency: int,
    max_long_vel: float,
    max_long_accel: float,
    init_pose: Pose5
) -> TrajectoryProfile:
    """
    Create poses and velocity commands to implement and measure accuracy of line trajectory with sinusoidally
    varying velocity. Full sinusoidal cycle returns robot back to starting point.

    Started with:
    v = (v_max)sin(wt)
    Integration of v leads to: 
    x = (v_max)(1 - cos(wt))
    Differentation of v leads to:
    a = w(v_max)cos(wt)

    Applying constraints on maximum longitudinal velocity and maximum longitudinal acceleration and leads to: 
    w = a_max / v_max
    
    Args:
        sample_frequency (int): Frequency (Hz) at which to sample analytical forms of trajectory position and velocity.
        max_long_vel (float): Maximum longitudinal (forward/reverse) velocity (m/s).
        max_long_accel (float): Maximum longitudinal (forward/reverse) acceleration (m/s^2). 
        init_pose (Pose5): Initial robot pose - needed to ensure that trajectory poses are relative to world frame.

    Returns TrajectoryProfile: Immutable dataclass storing tuples of poses, velocity commands, and longitudinal
        accelerations for sinusoidal line trajectory.
    """
    # Set angular frequency based on velocity and acceleration constraints
    angular_freq = max_long_accel / max_long_vel
    time_delta = 1 / sample_frequency
    num_samples = int(2 * pi / (angular_freq * time_delta)) + 1
    poses = []
    vels = []
    long_accels = []
    t = 0
    for _ in range(num_samples):
        x = max_long_vel * (1 - cos(angular_freq * t))
        v = max_long_vel * sin(angular_freq * t)
        a = angular_freq * max_long_vel * cos(angular_freq * t)

        # Build pose and include initial coordinates
        poses.append(Pose5(
            t=t,
            x=(init_pose.x + x),
            y=init_pose.y,  # Constant
            z=init_pose.z,  # Constant
            yaw=init_pose.yaw  # Constant
        ))

        vels.append(VelCmd(
            linear_x=v,
            angular_z=0
        ))

        long_accels.append(a)

        t += time_delta
        
    return TrajectoryProfile(
        tuple(poses),
        tuple(vels),
        tuple(long_accels)
    )

def circle(
    sample_frequency: int,
    max_angular_vel: float,
    max_lat_accel: float,
    init_pose: Pose5
) -> TrajectoryProfile:
    """
    Create poses and velocity commands to implement and measure accuracy of circle trajectory. 

    Analytical forms for circle:
    x = rcos(wt)
    y = rsin(wt)
    yaw = wt
    v = wr
    v_yaw = w 
    a_lat = v**2 / r

    Applying constraints on maximum angular velocity and maximum lateral acceleration and leads to: 
    r = max_lat_accel / (max_angular_vel ** 2)
    v = max_lat_accel / max_angular_vel
    a = w * v_max * cos(wt)
    
    Args:
        sample_frequency (int): Frequency (Hz) at which to sample analytical forms of trajectory position and velocity.
        max_angular_vel (float): Maximum angular (yaw) velocity (rad/s).
        max_lat_accel (float): Maximum lateral (perpendicular to heading) acceleration (m/s^2). 
        init_pose (Pose5): Initial robot pose - needed to ensure that trajectory poses are relative to world frame.

    Returns TrajectoryProfile: Immutable dataclass storing tuples of poses and velocity commands for circle trajectory.
    """
    # Set constant radius and velocity based on angular velocity and lateral acceleration constraints
    r = max_lat_accel / (max_angular_vel ** 2)
    v = max_lat_accel / max_angular_vel
    time_delta = 1 / sample_frequency
    num_samples = int(2 * pi / (max_angular_vel * time_delta)) + 1
    poses = []
    vels = []
    t = 0
    for _ in range(num_samples):
        yaw = max_angular_vel * t
        x = r * cos(yaw)
        y = r * sin(yaw)
        
        # Build pose and include initial coordinates
        poses.append(Pose5(
            t=t,
            x=(init_pose.x + x),
            y=(init_pose.y + y),
            z=init_pose.z,  # Constant
            yaw=(init_pose.yaw + yaw)
        ))

        vels.append(VelCmd(
            linear_x=v,  # Constant
            angular_z=max_angular_vel  # Constant
        ))

        t += time_delta
        
    return TrajectoryProfile(
        tuple(poses),
        tuple(vels)
    )


def figure_eight(
    sample_frequency: int,
    max_long_vel: float,
    max_angular_vel: float,
    max_long_accel: float,
    init_pose: Pose5,
    a: float = FIGURE_EIGHT_HALF_WIDTH,
    b: float = FIGURE_EIGHT_HALF_HEIGHT
) -> TrajectoryProfile:
    """
    Create poses and velocity commands to implement and measure accuracy of figure-of-eight trajectory. 

    Implement figure-of-eight using Lissajous-style curve:
    x = Acos(wt)
    y = Bsin(2wt)
    v_x = -wAsin(wt)
    v_y = 2wBcos(2wt)
    a_x = (-w**2)Acos(wt)
    a_y = -4(w**2)Bsin(2wt)
    yaw = atan2(v_y, v_x)
    v_yaw = ((v_x)(a_y) - (a_x)(v_y)) / (v_x**2 + v_y**2)

    To apply physical constraints to curve, a numerical method is used because it's not possible
    to find closed-form solutions relating w to max longitudinal velocity, max angular velocity,
    and max longitudinal acceleration. A and B are first chosen to determine the geometry of the
    curve, and then the angular frequency (or 'traversal speed'), w, can be scaled to satisfy 
    constraints. This works because the velocity, angular velocity, and acceleration are each
    periodic sin-cos combinations with their amplitudes dependent on w as follows:
    v = (w)sqrt((Asin(wt))**2 + (2Bcos(2wt))**2)
    v_yaw = (w)(4ABsin(wt)sin(2wt) + 2ABcos(wt)cos(2wt)) / ((Asin(wt))**2 + (Bcos(2wt))**2)
    a = (w**2)sqrt((Acos(wt))**2 + (4Bsin(2wt))**2)
    
    Args:
        sample_frequency (int): Frequency (Hz) at which to sample analytical forms of trajectory position and velocity.
        max_long_vel (float): Maximum longitudinal (forward/reverse) velocity (m/s).
        max_angular_vel (float): Maximum angular (yaw) velocity (rad/s).
        max_long_accel (float): Maximum longitudinal (forward/reverse) acceleration (m/s^2). 
        init_pose (Pose5): Initial robot pose - needed to ensure that trajectory poses are relative to world frame.
        a (float): Half-width of figure-eight curve.
        b (float): Half-height of figure-eight curve.

    Returns TrajectoryProfile: Immutable dataclass storing tuples of poses, velocity commands, and longitudinal
        accelerations for figure-eight trajectory.
    """
    time_delta = 1 / sample_frequency
    # Define initial arbitrary w 
    init_w = FIGURE_EIGHT_INITIAL_W
    init_num_samples = int(2 * pi / (init_w * time_delta)) + 1
    # Complete dummy run to build arrays with initial w
    init_v_mag_arr = []
    init_v_yaw_arr = []
    init_a_long_arr = []
    init_t = 0
    for _ in range(init_num_samples):
        v_x = -init_w * a * sin(init_w * init_t)
        v_y = 2 * init_w * b * cos(2 * init_w * init_t)
        v_mag = sqrt((v_x ** 2) + (v_y ** 2))

        a_x = -(init_w ** 2) * a * cos(init_w * init_t)
        a_y = -4 * (init_w ** 2) * b * sin(2 * init_w * init_t)

        yaw = atan2(v_y, v_x)
        v_yaw = ((v_x * a_y) - (a_x * v_y)) / (v_mag ** 2)

        # Get component of total acceleration along heading direction
        a_long = a_x * cos(yaw) + a_y * sin(yaw)

        init_v_mag_arr.append(v_mag)
        init_v_yaw_arr.append(v_yaw)
        init_a_long_arr.append(a_long)

        init_t += time_delta

    # Compute w scale factors to satisy constraints
    v_mag_factor = max_long_vel / max(init_v_mag_arr)  # Amplitude proportional to w
    v_yaw_factor = max_angular_vel / max(abs(v_yaw) for v_yaw in init_v_yaw_arr)  # Amplitude proportional to w
    a_factor = sqrt(max_long_accel / max(init_a_long_arr))  # Amplitude proportional to w**2

    # Scale w using smallest factor; this works for scaling down OR up and ensures all constraints
    # are satisifed. Will generally lead to conservative values for two of the constraints.
    w = init_w * min(v_mag_factor, v_yaw_factor, a_factor)

    # Compute poses and velocity commands
    num_samples = int(2 * pi / (w * time_delta)) + 1
    poses = []
    vels = []
    long_accels = []
    t = 0
    for _ in range(num_samples):    
        x = a * cos(w * t)
        y = b * sin(2 * w * t)

        v_x = -w * a * sin(w * t)
        v_y = 2 * w * b * cos(2 * w * t)
        v_mag = sqrt((v_x ** 2) + (v_y ** 2))

        a_x = -(w ** 2) * a * cos(w * t)
        a_y = -4 * (w ** 2) * b * sin(2 * w * t)

        yaw = atan2(v_y, v_x)
        v_yaw = ((v_x * a_y) - (a_x * v_y)) / (v_mag ** 2)

        # Get component of total acceleration along heading direction
        a_long = a_x * cos(yaw) + a_y * sin(yaw)

        # Build pose and include initial coordinates
        poses.append(Pose5(
            t=t,
            x=(init_pose.x + x),
            y=(init_pose.y + y),
            z=init_pose.z , # Constant
            yaw=(init_pose.yaw + yaw)
        ))

        vels.append(VelCmd(
            linear_x=v_mag,
            angular_z=v_yaw
        ))

        long_accels.append(a_long)

        t += time_delta

    return TrajectoryProfile(
        tuple(poses),
        tuple(vels),
        tuple(long_accels)
    )
