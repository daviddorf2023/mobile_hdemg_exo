import numpy as np

from mobile_hdemg_exo.calibration.trial import TrajectoryShape, TrialDirection, Trial


class TrajectoryGenerator:
    _trajectory_shape: TrajectoryShape
    _direction: TrialDirection
    _effort: float
    _duration: int
    _baseline_torque: int
    _MVC_torque: int

    def __init__(self,
                 trajectory_shape: TrajectoryShape,
                 direction: TrialDirection,
                 effort: float,
                 duration: int,
                 baseline_torque: int,
                 MVC_torque: int):
        self._trajectory_shape = trajectory_shape
        self._direction = direction
        self._effort = effort
        self._duration = duration
        self._baseline_torque = baseline_torque
        self._MVC_torque = MVC_torque

    @classmethod
    def from_trial(cls, trial: Trial):
        return cls(trial.trajectory_shape,
                   trial.direction,
                   trial.effort,
                   trial.duration,
                   trial.baseline_torque,
                   trial.MVC_torque)

    def generate(self):
        if self._trajectory_shape == TrajectoryShape.Flat:
            return self._generate_flat()
        elif self._trajectory_shape == TrajectoryShape.Trapezoid:
            return self._generate_trapezoid()
        elif self._trajectory_shape == TrajectoryShape.Sinusoid:
            return self._generate_sinusoid()
        elif self._trajectory_shape == TrajectoryShape.BidirectionalSinusoid:
            return self._generate_bidirectional_sinusoid()

    def _generate_flat(self):
        return np.zeros(self._duration).tolist()

    def _generate_trapezoid(self):
        min = self._baseline_torque
        max = self._MVC_torque
        if self._direction == TrialDirection.DF:
            max *= -1
        max = self._effort * (max - min)

        len = self._duration

        step = int(len / 5)
        desired_traj = []
        for i in range(0, int(.5 * step)):
            desired_traj.append(0)
        for i in range(int(.5 * step), 2 * step):
            y = (i * (max) / (step + int(.5 * step))) - max / 3  # 2*min - max
            desired_traj.append(y)
        for i in range(2 * step, 3 * step):
            desired_traj.append(max)
        for i in range(3 * step, int(4.5 * step)):
            y = ((-i * max) / (step + int(.5 * step))) + 3 * max  # - 3*min
            desired_traj.append(y)
        for i in range(int(4.5 * step), 5 * step):
            desired_traj.append(0)

        return desired_traj

    def _generate_sinusoid(self):
        min = self._baseline_torque
        max = self._MVC_torque
        if self._direction == TrialDirection.DF:
            max *= -1
        max = (self._effort * (max - min)) / 2

        len = self._duration

        step = int(len / 5)
        desired_traj = []
        for i in range(0, step):
            desired_traj.append((i * max / step))
        for i in range(step, 4 * step):
            desired_traj.append((max * np.sin(((2 * np.pi) / (3 * step)) * (i - step))) + max)
        for i in range(4 * step, 5 * step):
            desired_traj.append(-i * max / step + 5 * max)

        return desired_traj

    def _generate_bidirectional_sinusoid(self):
        min = self._baseline_torque
        max = self._MVC_torque
        if self._direction == TrialDirection.DF:
            max *= -1
        max = self._effort * (max - min)

        len = self._duration

        step = int(len / 5)
        desired_traj = []
        for i in range(0, int(.5 * step)):
            desired_traj.append(0)
        for i in range(int(.5 * step), int(4.5 * step)):
            desired_traj.append((max * np.sin(((2 * np.pi) / (4 * step)) * (i - .5 * step))))
        for i in range(int(4.5 * step), 5 * step):
            desired_traj.append(0)

        return desired_traj
