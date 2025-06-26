from dataclasses import dataclass

@dataclass(frozen=True)  # Immutable (frozen)
class Pose5:
    t: float  # s
    x: float  # m
    y: float  # m
    z: float  # m
    yaw: float  # rad 

@dataclass(frozen=True)  # Immutable (frozen)
class VelCmd:
    linear_x: float  # m/s
    angular_z: float  # rad/s
