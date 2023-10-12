#!/usr/bin/env python3

import rospy
import numpy as np
from scipy import signal
from std_msgs.msg import Float64, Float64MultiArray
from mobile_hdemg_exo.msg import StampedFloat64, StampedFloat64MultiArray
from mobile_hdemg_exo.processors.emg_process_cst import EMGProcessorCST
from mobile_hdemg_exo.streamer.emg_file_streamer import EMGFileStreamer
from mobile_hdemg_exo.streamer.emg_qc_streamer import EMGQCStreamer
from mobile_hdemg_exo.streamer.emg_muovi_streamer import EMGMUOVIStreamer
# from mobile_hdemg_exo.streamer.multi_muovi_streamer import EMGMUOVIStreamer
from mobile_hdemg_exo.utils.moving_average import MovingAverage
import RPi.GPIO as GPIO  # Latency analyzer dependency


while not rospy.get_param("gui_completed"):
    rospy.sleep(0.1)

# Launch file arguments
EMG_DEVICE = rospy.get_param("/device")
EMG_PROCESS_METHOD = rospy.get_param("/method")
LATENCY_ANALYZER_MODE = rospy.get_param("/latency_analyzer")
MUSCLE_COUNT = rospy.get_param("/muscle_count", int)
PWM_OUTPUT_PIN = rospy.get_param("/pwm_output_pin", int)
SAMPLING_FREQUENCY = rospy.get_param("/sampling_frequency", int)
NUM_TRIALS = rospy.get_param("/num_trials", int)
TRIAL_DURATION_SECONDS = 35 * NUM_TRIALS


class EMGStreamNode:
    """
    A class for streaming and processing EMG data.

    Attributes:
        streamer: An object that streams EMG data from the selected device.
        processor: An object that processes EMG data using the selected method.
        raw_pub: A ROS publisher for publishing raw EMG data.
        processed_pub: A ROS publisher for publishing processed EMG data.
        r: A ROS rate object for controlling the publishing rate.
    """

    def __init__(self):
        """
        Initializes the EMGStreamNode object.

        Sets up the ROS publishers and initializes the EMG streamer and processor. Also sets up objects for latency analysis, smoothing, and simulation.
        """
        rospy.init_node('emg_stream_node')
        self.start_time = rospy.get_time()
        self.streamer = None
        self.emg_pub = rospy.Publisher(
            'hdEMG_stream_processed', StampedFloat64, queue_size=10)
        self.array_emg_pub = rospy.Publisher(
            'hdEMG_stream_raw', StampedFloat64MultiArray, queue_size=10)
        self.imu_pub = rospy.Publisher(
            'imu_stream', StampedFloat64MultiArray, queue_size=10)
        self.old_reading = 0.
        self.moving_avg = MovingAverage(window_size=100)
        self.simulation_reading = np.zeros(70)

        # Initialize the PWM output pin
        if LATENCY_ANALYZER_MODE:
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(PWM_OUTPUT_PIN, GPIO.OUT, initial=GPIO.HIGH)
            self.p = GPIO.PWM(PWM_OUTPUT_PIN, 50)
            self.p.start(50)
        else:
            self.p = None

        # Initialize the EMG streamer
        if EMG_DEVICE == 'Quattrocento':
            self.streamer = EMGQCStreamer(MUSCLE_COUNT)
        elif EMG_DEVICE == 'Muovi+Pro':
            self.streamer = EMGMUOVIStreamer(MUSCLE_COUNT)
        elif EMG_DEVICE == 'File':
            self.path = rospy.get_param(
                "/file_dir") + "/src/mobile_hdemg_exo/new_emg_muovi_data1.csv"
            self.streamer = EMGFileStreamer(
                MUSCLE_COUNT, SAMPLING_FREQUENCY, self.path)
        elif EMG_DEVICE == 'SimMuovi+Pro':
            # Subscribe to the /hdEMG_SimMuovi+Pro topic
            rospy.Subscriber('/hdEMG_Simulation',
                             Float64MultiArray, self.simulation_callback)
        else:
            raise ValueError('Invalid EMG_DEVICE')

        # Match the streamer's publishing rate
        if EMG_DEVICE != 'SimMuovi+Pro':
            self.r = rospy.Rate(SAMPLING_FREQUENCY)
            self.streamer.initialize()

    def simulation_callback(self, data):
        """
        Callback function for the /hdEMG_Simulation topic.

        Args:
            data: A list of floats representing EMG data.
        """
        self.simulation_reading = data.data

    def notch_filter(self, data):
        """ 
        Applies a notch filter to the EMG data.

        Args:
            data: A list of integers representing an EMG reading.

        Returns:
            A list of integers representing an EMG reading with a notch filter applied.
        """
        b, a = signal.iirnotch(60, 30, SAMPLING_FREQUENCY)
        return signal.filtfilt(b, a, data)

    def butter_bandpass(self, lowcut, highcut, fs, order=5):
        """ 
        Creates a bandpass filter.

        Args:
            lowcut: The lower cutoff frequency.
            highcut: The higher cutoff frequency.
            fs: The sampling frequency.
            order: The order of the filter.

        Returns:
            A list of integers representing an EMG reading with a bandpass filter applied.
        """
        nyq = 0.5 * fs
        low = lowcut / nyq
        high = highcut / nyq
        b, a = signal.butter(order, [low, high], btype='band')
        return b, a

    def publish_reading(self, publisher: rospy.topics.Publisher, reading: float):
        """
        Publishes the EMG reading to a ROS topic.

        Args:
            publisher: A ROS publisher object.
            reading: A list of integers representing an EMG reading.
        """
        message = StampedFloat64()
        message.header.stamp = rospy.get_rostime()
        message.data = Float64(data=reading)
        publisher.publish(message)

    def pwm_cleanup(self):
        self.p.stop()
        GPIO.cleanup()

    def quaternion_to_roll_pitch_yaw(self, quaternion):
        """
        Converts a quaternion to roll, pitch, and yaw angles.

        Args:
            quaternion: A list of four floats representing a quaternion.

        Returns:
            A list of three floats representing roll, pitch, and yaw angles.
        """
        w, x, y, z = quaternion
        # Normalize the quaternion
        mag = np.sqrt(w**2 + x**2 + y**2 + z**2)
        w /= mag
        x /= mag
        y /= mag
        z /= mag
        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        pitch = np.arcsin(2 * (w * y - z * x))
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        return roll, pitch, yaw

    def run_emg(self):
        """
        Runs the EMG streamer and processor.

        Reads EMG data from the streamer, processes it using the selected method, and publishes the raw and processed data
        to ROS topics.
        """
        if EMG_DEVICE != 'SimMuovi+Pro':
            raw_reading = self.streamer.stream_data()
        else:
            raw_reading = self.simulation_reading

        # Device-specific processing
        if EMG_DEVICE == 'Quattrocento':
            offset = 32 * MUSCLE_COUNT
            # Each MULTIPLE IN has 64 channels
            hdemg_reading = raw_reading[offset:offset + MUSCLE_COUNT * 64]
        elif EMG_DEVICE == 'Muovi+Pro' or EMG_DEVICE == 'SimMuovi+Pro':
            # Each Muovi+ EMG probe has 70 channels. After 64, 4 are quaternion then 2 are debug
            hdemg_reading = raw_reading[:MUSCLE_COUNT * 64]
            # Publish IMU data
            imu_reading = raw_reading[MUSCLE_COUNT * 64:MUSCLE_COUNT * 64 + 4]
            roll, pitch, yaw = self.quaternion_to_roll_pitch_yaw(imu_reading)
            imu_msg = StampedFloat64MultiArray()
            imu_msg.header.stamp = rospy.get_rostime()
            imu_msg.data = Float64MultiArray(data=[roll, pitch, yaw])
            self.imu_pub.publish(imu_msg)
        elif EMG_DEVICE == 'File':
            hdemg_reading = raw_reading
        else:
            raise ValueError('Invalid EMG_DEVICE')

        # Method-specific processing
        if LATENCY_ANALYZER_MODE and EMG_DEVICE == 'Quattrocento':
            processed_emg = hdemg_reading[96]  # Auxiliary channel 1
        elif LATENCY_ANALYZER_MODE and EMG_DEVICE == 'Muovi+Pro':
            processed_emg = hdemg_reading[-1]  # Last channel is auxiliary
        elif EMG_PROCESS_METHOD == 'RMS':
            processed_emg = (np.mean(hdemg_reading ** 2))**0.5
            # Apply moving average filter
            self.moving_avg.add_data_point(processed_emg)
            smooth_emg = self.moving_avg.get_smoothed_value()
            # Apply notch filter
            hdemg_reading = self.notch_filter(hdemg_reading)
            # Apply bandpass filter
            b, a = self.butter_bandpass(20, 500, SAMPLING_FREQUENCY)
            hdemg_reading = signal.filtfilt(b, a, hdemg_reading)
            # Publish the processed EMG data
            self.publish_reading(self.emg_pub, smooth_emg)
        elif EMG_PROCESS_METHOD == 'CST':
            self.processor = EMGProcessorCST()
            processed_emg = self.processor.process_reading(hdemg_reading)
            self.moving_avg.add_data_point(processed_emg)
            smooth_emg = self.moving_avg.get_smoothed_value()
            self.processor.publish_reading(self.emg_pub, smooth_emg)
        elif EMG_PROCESS_METHOD == 'Raw':
            # Add 1 through 64 to the raw EMG data to make it easier to visualize
            spacing = 10
            hdemg_reading = hdemg_reading + spacing * np.arange(1, 65)
            raw_message = StampedFloat64MultiArray()
            raw_message.header.stamp = rospy.get_rostime()
            raw_message.data = Float64MultiArray(data=hdemg_reading)
            self.array_emg_pub.publish(raw_message)
        else:
            raise ValueError('Invalid EMG_PROCESS_METHOD')

        # Publish processed data at the same rate as the streamer
        self.r.sleep()


if __name__ == '__main__':
    emg_stream_node = EMGStreamNode()
    while not rospy.is_shutdown():
        emg_stream_node.run_emg()
    if LATENCY_ANALYZER_MODE:
        emg_stream_node.pwm_cleanup()
    emg_stream_node.streamer.close()
