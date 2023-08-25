from enum import Enum


class TrialDirection(Enum):
    PF = "PF"
    DF = "DF"
    NoDirection = "NoDirection"


class TrajectoryShape(Enum):
    Flat = "flat",
    Trapezoid = "trapezoid",
    Sinusoid = "sinusoid",
    BidirectionalSinusoid = "bidirectional-sinusoid"


class Trial:
    joint_angle: float
    direction: TrialDirection
    trajectory_shape: TrajectoryShape
    effort: float
    duration: int

    min_torque: int
    baseline_torque: int
    MVC_torque: int

    emg_array = []
    torque_array = []

    def __init__(self,
                 joint_angle: float,
                 direction: TrialDirection,
                 trajectory_shape: TrajectoryShape,
                 effort: float,
                 duration: int):
        self.joint_angle = joint_angle
        self.direction = direction
        self.trajectory_shape = trajectory_shape
        self.effort = effort
        self.duration = duration
