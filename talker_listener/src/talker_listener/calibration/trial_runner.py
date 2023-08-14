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

import tkinter as tk
import pyttsx3
import rospy


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
        self.side_id = rospy.get_param("/side_id")

        # Create a tkinter window
        self.window = tk.Tk()

        # Set the window title
        self.window.title("Shirley Ryan AbilityLab - Patient Instruction for Ankle Exoskeleton")

        # Set the window to fullscreen
        self.window.attributes('-fullscreen', True)

        # Create a label to display the messages
        self.message_label = tk.Label(self.window, font=("Calibri", 80), pady=20)
        self.message_label.pack(expand=True)

        # Initialize the text-to-speech engine
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 150)
        self.engine.setProperty('volume', 1.0)
        self.voices = self.engine.getProperty('voices')
        self.engine.setProperty('voice', self.voices[2].id)

    def __enter__(self):
        # Subscribers for the torque and hd-EMG publishers
        self._torque_sub = rospy.Subscriber('/h3/robot_states', State,
                                            lambda x: self._torque_smoother.process_reading(x.joint_torque_sensor[self.side_id]))
        self._emg_sub = rospy.Subscriber('hdEMG_stream_processed', hdemg,
                                         lambda x: self._emg_array.append(x.data.data))
        self._timescale_sub = rospy.Subscriber('/h3/robot_states', State,
                                               lambda x: self._timescale.timestamp())

        # Publisher for position control
        if (self.side_id == 2):
            self._position_pub = rospy.Publisher('/h3/right_ankle_position_controller/command', Float64, queue_size=0)
        elif (self.side_id == 5):
            self._position_pub = rospy.Publisher('/h3/left_ankle_position_controller/command', Float64, queue_size=0)
        elif (self.side_id == 1):
            self._position_pub = rospy.Publisher('/h3/right_ankle_position_controller/command', Float64, queue_size=0) # arbitrary, side_id 1 used for simulation
        else:
            raise NameError("Side ID must be 1 [sim], 2 [right ankle], or 5 [left ankle]")
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
                axs.plot(self._timescale.axis, self._offset_torque_array, color='red')
                plt.pause(.01)
                self._r.sleep()

            trial.emg_array = self._emg_array.copy()
            trial.torque_array = self._torque_array.copy()
            plt.close()
    
    def update_gui(self, message):
        self.engine.say(message)
        self.engine.runAndWait()
        self.message_label.config(text=message)
        self.message_label.update()
        self.window.update()

    def _collect_baseline_torque(self):
        print("Collecting baseline torque...")

        self._reset_measures()
        message = "Please relax your foot"
        self.update_gui(message)

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
        message = "Please press your foot down"
        self.update_gui(message)
        print("GO!")

        while countdown.is_time_left():
            self._r.sleep()
        mvc1 = np.max(np.abs(self._torque_array))
        print(f"MVC1: {mvc1}")

        message = "Please relax your foot"
        self.update_gui(message)
        print("REST")
        rospy.sleep(5)

        self._reset_measures()
        countdown.reset()

        message = "Please lift your foot up"
        self.update_gui(message)
        print("GO!")
        while countdown.is_time_left():
            self._r.sleep()
        mvc2 = np.max(np.abs(self._torque_array))
        print(f"MVC2: {mvc1}")

        message = "Please relax your foot"
        self.update_gui(message)
        print("REST")
        rospy.sleep(5)

        # Close the window when finished
        self.window.destroy()

        # Start the tkinter event loop
        self.window.mainloop()

        return np.average([mvc1, mvc2])

    def _set_exo_angle(self, angle):
        print("Moving to {} degrees".format(str(np.rad2deg(angle))))
        self._position_pub.publish(float(angle))
        rospy.sleep(5)

    def _reset_measures(self):
        self._emg_array.clear()
        self._torque_smoother.reset()
        self._timescale.reset()
