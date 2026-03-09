from dataclasses import dataclass, field


@dataclass(frozen=True)  # Immutable (frozen)
class Pose5:
    t: float  # s
    x: float  # m
    y: float  # m
    z: float  # m
    yaw: float  # rad 


@dataclass(frozen=True)  # Immutable
class VelCmd:
    linear_x: float  # m/s
    angular_z: float  # rad/s


@dataclass(frozen=True)  # Immutable
class TrajectoryProfile:
    poses: tuple[Pose5, ...]
    vels: tuple[VelCmd, ...]  
    long_accels: tuple[float, ...] = field(default_factory=tuple)  # Empty tuple default 
