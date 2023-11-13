#!/usr/bin/env python3

import rospy
import numpy as np
from scipy import signal
from std_msgs.msg import Float64, Float64MultiArray
from mobile_hdemg_exo.processors.emg_process_cst import EMGProcessorCST
from mobile_hdemg_exo.msg import StampedFloat64, StampedFloat64MultiArray
from mobile_hdemg_exo.streamer.emg_file_streamer import EMGFileStreamer
from mobile_hdemg_exo.streamer.emg_qc_streamer import EMGQCStreamer
from mobile_hdemg_exo.streamer.emg_muovi_streamer import EMGMUOVIStreamer
from mobile_hdemg_exo.utils.moving_average import MovingAverage

EMG_DEVICE = rospy.get_param("/device")
EMG_PROCESS_METHOD = rospy.get_param("/method")
MUSCLE_COUNT = rospy.get_param("/muscle_count", int)
SAMPLING_FREQUENCY = rospy.get_param("/sampling_frequency", int)
LATENCY_ANALYZER_MODE = rospy.get_param("/latency_analyzer")
PWM_OUTPUT_PIN = rospy.get_param("/pwm_output_pin", int)
SPACING = 30  # Spacing between channels for visualization


while not rospy.get_param("gui_completed"):
    rospy.sleep(0.1)


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
        self.receive_flag = False

        # Initialize the PWM output pin
        if LATENCY_ANALYZER_MODE:
            import RPi.GPIO as GPIO
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
                "/file_dir") + "/data/cst_test_data_nov1/raw_emg_7.csv"
            self.streamer = EMGFileStreamer(
                MUSCLE_COUNT, SAMPLING_FREQUENCY, self.path)
        elif EMG_DEVICE == 'SimMuovi+Pro':
            rospy.Subscriber('/hdEMG_Simulation',
                             Float64MultiArray, self.simulation_callback)
        else:
            raise ValueError('Invalid EMG_DEVICE')

        if EMG_PROCESS_METHOD == 'CST':
            self.processor = EMGProcessorCST()

        # Match the streamer's publishing rate
        self.r = rospy.Rate(SAMPLING_FREQUENCY)

        # Initialize the EMG streamer
        if EMG_DEVICE != 'SimMuovi+Pro':
            self.streamer.initialize()
            raw_reading = self.streamer.stream_data()
        else:
            raw_reading = self.simulation_reading

        self.hdemg_reading = None
        if EMG_DEVICE == 'Quattrocento':
            offset = 32 * MUSCLE_COUNT
            self.hdemg_reading = raw_reading[offset:offset + MUSCLE_COUNT * 64]
        elif EMG_DEVICE == 'Muovi+Pro' or EMG_DEVICE == 'SimMuovi+Pro':
            self.hdemg_reading = raw_reading[:MUSCLE_COUNT * 64]
        elif EMG_DEVICE == 'File':
            self.hdemg_reading = raw_reading[128: MUSCLE_COUNT * 64 + 128]
        else:
            raise ValueError('Invalid EMG_DEVICE')
        
        if self.hdemg_reading is None:
            print('No data available yet')
            rospy.sleep(0.1)
        print('EMG data received')

        self.removed_channels = rospy.get_param("/channels_to_remove")
        if self.removed_channels != '':
            self.removed_channels = self.removed_channels.split(',')
            self.removed_channels = list(map(int, self.removed_channels))
            self.removed_channels = [
                x for x in self.removed_channels if x < MUSCLE_COUNT * 64]

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

    def smooth_emg(self, emg_reading):
        """
        Smoothes the EMG reading.

        Args:
            emg_reading: A list of integers representing an EMG reading.

        Returns:
            A list of integers representing a smoothed EMG reading.
        """
        self.moving_avg.add_data_point(emg_reading)
        return self.moving_avg.get_smoothed_value()

    def csv_output(self, emg_reading):
        """
        Outputs the EMG reading to a CSV file.

        Args:
            emg_reading: A list of integers representing an EMG reading.
        """
        with open('emg_data.csv', 'a') as f:
            f.write(str(emg_reading) + '\n')

    def run_emg(self):
        """
        Runs the EMG streamer and processor.

        Reads EMG data from the streamer, processes it using the selected method, and publishes the raw and processed data
        to ROS topics.
        """
        # Publish raw EMG data
        raw_hdemg_reading = self.hdemg_reading + \
            SPACING * np.arange(len(self.hdemg_reading))
        raw_hdemg_reading = raw_hdemg_reading / SPACING  # Scale to channel numbers
        raw_hdemg_reading = np.delete(
            raw_hdemg_reading, self.removed_channels)
        raw_message = StampedFloat64MultiArray()
        raw_message.header.stamp = rospy.get_rostime().from_sec(
            rospy.get_time() - self.start_time)
        raw_message.data = Float64MultiArray(data=raw_hdemg_reading)
        self.array_emg_pub.publish(raw_message)

        # Publish processed EMG data
        notch_reading = self.notch_filter(self.hdemg_reading)
        b, a = self.butter_bandpass(20, 100, SAMPLING_FREQUENCY)
        hdemg_filtered = signal.filtfilt(b, a, notch_reading)
        if EMG_PROCESS_METHOD == 'RMS':
            for channel in self.removed_channels:
                hdemg_filtered[channel] = np.mean(hdemg_filtered)
            rms_emg = (np.mean(np.array(hdemg_filtered)**2))**0.5
            smooth_emg = self.smooth_emg(rms_emg)
            self.publish_reading(self.emg_pub, smooth_emg)
            self.csv_output(smooth_emg)
        elif EMG_PROCESS_METHOD == 'CST':
            for channel in self.removed_channels:
                hdemg_filtered[channel] = 0
            processed_emg = self.processor.process_reading(hdemg_filtered/100)
            smooth_emg = self.smooth_emg(processed_emg)
            self.publish_reading(self.emg_pub, processed_emg)
            self.csv_output(smooth_emg)
        else:
            raise ValueError('Invalid EMG_PROCESS_METHOD')
        self.r.sleep()


if __name__ == '__main__':
    emg_stream_node = EMGStreamNode()
    while not rospy.is_shutdown():
        emg_stream_node.run_emg()
    if LATENCY_ANALYZER_MODE:
        emg_stream_node.pwm_cleanup()
    if EMG_DEVICE != 'SimMuovi+Pro':
        emg_stream_node.streamer.close()
