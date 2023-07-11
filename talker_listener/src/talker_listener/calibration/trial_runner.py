import numpy as np
import rospy
from h3_msgs.msg import State
from matplotlib import pyplot as plt
from std_msgs.msg import Float64
from talker_listener.msg import hdemg

from talker_listener.calibration.trajectory_generator import TrajectoryGenerator
from talker_listener.calibration.trial import Trial, TrialDirection
from talker_listener.utils.rospy_countdown import RospyCountdown
from talker_listener.utils.timescale_axis import TimescaleAxis
from talker_listener.utils.torque_smoother import TorqueSmoother


class TrialRunner:
    _r: rospy.Rate

    _emg_sub: rospy.Subscriber
    _torque_sub: rospy.Subscriber
    _timescale_sub: rospy.Subscriber
    _position_pub: rospy.Publisher

    _emg_array = []
    _torque_smoother: TorqueSmoother
    _timescale: TimescaleAxis
    trials: [Trial] = []

    def __init__(self, trials: [Trial]):
        self.trials = trials
        self._r = rospy.Rate(2048)
        self._torque_smoother = TorqueSmoother()
        self._timescale = TimescaleAxis()

    def __enter__(self):
        # Subscribers for the torque and hd-EMG publishers
        self._torque_sub = rospy.Subscriber('/h3/robot_states', State,
                                            lambda x: self._torque_smoother.process_reading(x.joint_torque_sensor[1]))
        self._emg_sub = rospy.Subscriber('hdEMG_stream_processed', hdemg,
                                         lambda x: self._emg_array.append(x.data.data))
        self._timescale_sub = rospy.Subscriber('/h3/robot_states', State,
                                               lambda x: self._timescale.timestamp())

        # Publisher for position control
        self._position_pub = rospy.Publisher('/h3/right_ankle_position_controller/command', Float64, queue_size=0)
        return self

    def __exit__(self, *args):
        self._torque_sub.unregister()
        self._emg_sub.unregister()
        self._position_pub.unregister()

    @property
    def _torque_array(self):
        return self._torque_smoother.torque_array

    @property
    def _offset_torque_array(self):
        return self._torque_smoother.offset_torque_array

    def collect_trial_data(self):
        if self._torque_sub is None or self._emg_sub is None or self._position_pub is None:
            raise NameError("One of self.torque_sub, self.emg_sub, self.pos_pub is None. Cannot run calibrate()")

        for trial in self.trials:
            self._set_exo_angle(trial.joint_angle)

            baseline_torque, min_torque = self._collect_baseline_torque()
            trial.baseline_torque = baseline_torque
            trial.min_torque = min_torque

            if trial.direction != TrialDirection.NoDirection:
                print(f"Apply Max {trial.direction.value} torque")
                trial.MVC_torque = self._collect_max_torque()
            else:
                trial.MVC_torque = 2.0

            reference_trajectory = TrajectoryGenerator.from_trial(trial).generate()
            fig, axs = plt.subplots()
            axs.set_xlim(0, trial.duration)
            y_lim = 1.5 * trial.effort * trial.MVC_torque
            axs.set_ylim(-1 * y_lim, y_lim)
            axs.plot(reference_trajectory, color='blue')
            plt.pause(0.01)

            self._reset_measures()
            self._torque_smoother.offset = baseline_torque
            countdown = RospyCountdown(rospy.Duration.from_sec(trial.duration))
            while countdown.is_time_left():
                # TODO: is self._torque_array - baseline_torque faster than
                # torque_smoother tracking offset_torque_array?
                axs.plot(self._timescale.axis, self._offset_torque_array, color='red')
                plt.pause(.01)
                self._r.sleep()

            trial.emg_array = self._emg_array.copy()
            trial.torque_array = self._torque_array.copy()
            plt.close()

    def _collect_baseline_torque(self):
        print("Collecting baseline torque...")

        self._reset_measures()

        print("REST")
        rospy.sleep(5)
        baseline_torque = np.median(self._torque_array)
        min_torque = np.min(np.abs(self._torque_array))
        print(f"Collected baseline_torque={baseline_torque} and min_torque={min_torque}")

        return baseline_torque, min_torque

    def _collect_max_torque(self):
        print("Collecting max torque...")

        self._reset_measures()
        countdown = RospyCountdown(rospy.Duration.from_sec(5))

        print("GO!")
        while countdown.is_time_left():
            self._r.sleep()
        mvc1 = np.max(np.abs(self._torque_array))
        print(f"MVC1: {mvc1}")

        print("REST")
        rospy.sleep(5)

        self._reset_measures()
        countdown.reset()

        print("GO!")
        while countdown.is_time_left():
            self._r.sleep()
        mvc2 = np.max(np.abs(self._torque_array))
        print(f"MVC2: {mvc1}")

        print("REST")
        rospy.sleep(5)

        return np.average([mvc1, mvc2])

    def _set_exo_angle(self, angle):
        print("Moving to {} degrees".format(str(np.rad2deg(angle))))
        self._position_pub.publish(float(angle))
        rospy.sleep(5)

    def _reset_measures(self):
        self._emg_array.clear()
        self._torque_smoother.reset()
        self._timescale.reset()
