import numpy as np
import rospy
from h3_msgs.msg import State
from matplotlib import pyplot as plt
from std_msgs.msg import Float64
from mobile_hdemg_exo.msg import hdemg
from mobile_hdemg_exo.calibration.trajectory_generator import TrajectoryGenerator
from mobile_hdemg_exo.calibration.trial import Trial, TrialDirection
from mobile_hdemg_exo.utils.rospy_countdown import RospyCountdown
from mobile_hdemg_exo.utils.timescale_axis import TimescaleAxis
import tkinter as tk
import pyttsx3
import rospy

# TODO: Generalize for 3 muscles
# TODO: Reimplement MVC calculation and trajectory generation


class TrialRunner:
    _r: rospy.Rate

    _emg_sub: rospy.Subscriber
    _torque_sub: rospy.Subscriber
    _timescale_sub: rospy.Subscriber
    _position_pub: rospy.Publisher
    _timescale: TimescaleAxis
    _battery_sub: rospy.Subscriber
    trials: [Trial] = []

    def __init__(self, trials: [Trial]):
        self.trials = trials
        self._r = rospy.Rate(100)
        self._timescale = TimescaleAxis()
        self._emg_array = []
        self._emg_time_array = []
        self._torque_array = []
        self._torque_time_array = []
        self._battery_voltage = []
        self._imu_array = []
        self._imu_time_array = []
        self.side = rospy.get_param("/side")
        self.device = rospy.get_param("/device")

        # Enumerate sides for indexing the torque sensor
        if self.side == "Left":
            self.side_id = 5
        if self.side == "Right":
            self.side_id = 2
        if self.side == "Simulation":
            self.side_id = 1

        # Subscribers for the torque and hd-EMG publishers
        self._torque_sub = rospy.Subscriber(
            '/h3/robot_states', State, self.torque_callback)
        self._emg_sub = rospy.Subscriber(
            '/hdEMG_stream_processed', hdemg, self.emg_callback)
        self._imu_sub = rospy.Subscriber(
            '/imu_stream', hdemg, self.imu_callback)
        self._battery_sub = rospy.Subscriber(
            '/h3/robot_states', State, self.battery_callback)

        # Publisher for position control
        if (self.side == "Left"):
            self._position_pub = rospy.Publisher(
                '/h3/left_ankle_position_controller/command', Float64, queue_size=0)
        elif (self.side == "Right"):
            self._position_pub = rospy.Publisher(
                '/h3/right_ankle_position_controller/command', Float64, queue_size=0)
        elif (self.device == "Simulation"):
            self._position_pub = rospy.Publisher(
                '/h3/right_ankle_position_controller/command', Float64, queue_size=0)
        else:
            raise NameError(
                "Side name must be Left, Right, or the system must be in Simulation device mode")

        # Create a tkinter window
        self.window = tk.Tk()

        # Set the window title
        self.window.title(
            "Shirley Ryan AbilityLab - Patient Instruction for Ankle Exoskeleton")

        # Set the window to fullscreen
        self.window.attributes('-fullscreen', True)

        # Create a label to display the messages
        self.message_label = tk.Label(
            self.window, font=("Calibri", 80), pady=20)
        self.message_label.pack(expand=True)

        # Initialize the text-to-speech engine
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 100)
        self.engine.setProperty('volume', 1.0)
        self.voices = self.engine.getProperty('voices')
        self.engine.setProperty('voice', self.voices[2].id)

    def emg_callback(self, data):
        self._emg_array.append(data.data.data)
        self._emg_time_array.append(data.header.stamp.to_sec())

    def torque_callback(self, data):
        self._torque_array.append(data.joint_torque_sensor[self.side_id])
        self._torque_time_array.append(data.header.stamp.to_sec())

    def battery_callback(self, data):
        if data.battery_voltage < 18.0 and data.battery_voltage > 1:
            print(data.battery_voltage)
            print("Please charge the battery")

    def imu_callback(self, data):
        self._imu_array.append(data.data.data)
        self._imu_time_array.append(data.header.stamp.to_sec())

    def collect_trial_data(self):
        if self._torque_sub is None or self._emg_sub is None or self._position_pub is None:
            raise NameError(
                "One of self.torque_sub, self.emg_sub, self.pos_pub is None. Cannot run calibrate()")

        for trial in self.trials:
            self._set_exo_angle(trial.joint_angle)
            baseline_torque, min_torque = self._collect_baseline_torque()
            trial.baseline_torque = baseline_torque
            trial.min_torque = min_torque

            if trial.direction != TrialDirection.NoDirection:
                trial.MVC_torque = self._collect_max_torque()
            else:
                trial.MVC_torque = 2.0

            # Plot the torque and hd-EMG data with the same time axis
            plt.plot(self._torque_time_array,
                     self._torque_array, label='Torque')
            plt.plot(self._emg_time_array, self._emg_array, label='EMG')
            plt.xlabel('Time (s)')
            plt.ylabel('Torque (Nm) / EMG (mV)')
            plt.title('Torque Sensor and EMG Data')
            plt.legend()
            plt.show()

            # Save the trial data
            trial.emg_array = self._emg_array.copy()
            trial.torque_array = self._torque_array.copy()

            # Calculate the EMG coefficients from the EMG data and torque data
            emg_array = np.array(trial.emg_array)
            emg_array = emg_array[~np.isnan(emg_array)]
            torque_array = np.array(trial.torque_array)

            emg_min = np.min(emg_array)
            emg_avg = np.average(emg_array)
            emg_max = np.max(emg_array)

            torque_max = np.max(torque_array)
            torque_min = np.min(torque_array)

            emg_coef_up = torque_min / emg_min
            emg_coef_down = torque_max / emg_max

            rospy.set_param('emg_coef_up', float(emg_coef_up))
            rospy.set_param('emg_avg', float(emg_avg))
            rospy.set_param('emg_coef_down', float(emg_coef_down))

            rospy.set_param("calibrated", True)

            self._reset_measures()

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
        print(
            f"Collected baseline_torque={baseline_torque} and min_torque={min_torque}")

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
        # TODO: Reimplement with separate arrays for each part of the trial, as well as whole trial
        # self._torque_array.clear()
        # self._emg_array.clear()
        # self._torque_time_array.clear()
        # self._emg_time_array.clear()
        return 0
