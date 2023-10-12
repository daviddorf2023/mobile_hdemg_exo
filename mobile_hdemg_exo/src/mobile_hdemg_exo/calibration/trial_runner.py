import numpy as np
import rospy
from h3_msgs.msg import State
from std_msgs.msg import Float64
from mobile_hdemg_exo.msg import StampedFloat64MultiArray, StampedFloat64
from matplotlib import pyplot as plt
from mobile_hdemg_exo.calibration.trial import Trial, TrialDirection
import tkinter as tk
import pyttsx3
import rospy


class TrialRunner:
    _r: rospy.Rate
    _emg_sub: rospy.Subscriber
    _torque_sub: rospy.Subscriber
    _battery_sub: rospy.Subscriber
    _position_pub: rospy.Publisher
    trials: [Trial] = []

    def __init__(self, trials: [Trial]):
        self.trials = trials
        self._r = rospy.Rate(100)
        self._emg_array = []
        self._emg_time_array = []
        self._torque_array = []
        self._MVC_torque_array = []
        self._torque_time_array = []
        self._battery_voltage = []
        self._imu_roll = []
        self._imu_pitch = []
        self._imu_yaw = []

        # For when quaternion data is stored in the imu message instead of roll, pitch, yaw
        # self._imu_w = []
        # self._imu_x = []
        # self._imu_y = []
        # self._imu_z = []

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
            '/hdEMG_stream_processed', StampedFloat64, self.emg_callback)
        self._imu_sub = rospy.Subscriber(
            '/imu_stream', StampedFloat64MultiArray, self.imu_callback)
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
        self._MVC_torque_array.append(data.joint_torque_sensor[self.side_id])

    def imu_callback(self, data):
        self._imu_roll.append(data.data.data[0])
        self._imu_pitch.append(data.data.data[1])
        self._imu_yaw.append(data.data.data[2])
        # self._imu_w.append(data.data.data[0])
        # self._imu_x.append(data.data.data[1])
        # self._imu_y.append(data.data.data[2])
        # self._imu_z.append(data.data.data[3])
        self._imu_time_array.append(data.header.stamp.to_sec())

    def battery_callback(self, data):
        if data.battery_voltage < 18.0 and data.battery_voltage > 1:
            print("Please charge the battery" +
                  f"Battery voltage: {data.battery_voltage}")

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

            # Plot the IMU data
            plt.plot(self._imu_time_array, self._imu_roll, label='Roll')
            plt.plot(self._imu_time_array, self._imu_pitch, label='Pitch')
            plt.plot(self._imu_time_array, self._imu_yaw, label='Yaw')
            plt.xlabel('Time (s)')
            plt.ylabel('Angle (rad)')
            plt.title('IMU Data')
            # plt.plot(self._imu_time_array, self._imu_w, label='w')
            # plt.plot(self._imu_time_array, self._imu_x, label='x')
            # plt.plot(self._imu_time_array, self._imu_y, label='y')
            # plt.plot(self._imu_time_array, self._imu_z, label='z')
            plt.legend()
            plt.show()

            # Save the data to a file
            np.savetxt("torque_data.csv", self._torque_array, delimiter=",")
            np.savetxt("emg_data.csv", self._emg_array, delimiter=",")
            np.savetxt("torque_time.csv",
                       self._torque_time_array, delimiter=",")
            np.savetxt("emg_time.csv", self._emg_time_array, delimiter=",")

            # Calculate the calibration coefficient
            emg_avg = np.average(self._torque_array) / \
                np.average(self._emg_array)
            torque_avg = np.average(self._emg_array) / \
                np.average(self._torque_array)
            rospy.set_param('emg_coef', (float)(torque_avg/emg_avg))
            rospy.set_param("calibrated", True)

    def update_gui(self, message):
        self.engine.say(message)
        self.engine.runAndWait()
        self.message_label.config(text=message)
        self.message_label.update()
        self.window.update()

    def _collect_baseline_torque(self):
        print("Collecting baseline torque...")
        self._MVC_torque_array = []
        message = "Please relax your foot"
        self.update_gui(message)
        rospy.sleep(5)
        baseline_torque = np.median(self._torque_array)
        min_torque = np.min(np.abs(self._torque_array))
        print(
            f"Collected baseline_torque={baseline_torque} and min_torque={min_torque}")
        return baseline_torque, min_torque

    def _collect_max_torque(self):
        print("Collecting max torque...")
        self._MVC_torque_array = []

        message = "Please press your foot down"
        self.update_gui(message)
        rospy.sleep(5)
        mvc1 = np.max(np.abs(self._torque_array))
        print(f"MVC1: {mvc1}")

        message = "Please relax your foot"
        self.update_gui(message)
        rospy.sleep(5)
        self._MVC_torque_array = []

        message = "Please lift your foot up"
        self.update_gui(message)
        rospy.sleep(5)
        mvc2 = np.max(np.abs(self._torque_array))
        print(f"MVC2: {mvc1}")

        message = "Please relax your foot"
        self.update_gui(message)
        rospy.sleep(5)
        self._MVC_torque_array = []

        # Close the window when finished
        self.window.destroy()
        self.window.mainloop()

        return np.average([mvc1, mvc2])

    def _set_exo_angle(self, angle):
        print("Moving to {} degrees".format(str(np.rad2deg(angle))))
        self._position_pub.publish(float(angle))
        rospy.sleep(5)
