#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray
from mobile_hdemg_exo.msg import hdemg, imu
from mobile_hdemg_exo.processors.emg_process_cst import EMGProcessorCST
from mobile_hdemg_exo.streamer.emg_file_streamer import EMGFileStreamer
from mobile_hdemg_exo.streamer.emg_qc_streamer import EMGQCStreamer
from mobile_hdemg_exo.streamer.emg_muovi_streamer import EMGMUOVIStreamer
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

        Sets up the ROS publishers and initializes the EMG streamer and processor.
        """
        rospy.init_node('emg_stream_node')
        self.start_time = rospy.get_time()
        self.streamer = None
        self.emg_pub = rospy.Publisher(
            'hdEMG_stream_processed', hdemg, queue_size=10)
        self.imu_pub = rospy.Publisher('imu_stream', imu, queue_size=10)
        self.old_reading = 0.

        # Initialize the PWM output pin
        if LATENCY_ANALYZER_MODE:
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(PWM_OUTPUT_PIN, GPIO.OUT, initial=GPIO.HIGH)
            self.p = GPIO.PWM(PWM_OUTPUT_PIN, 50)
            self.p.start(50)

        # Initialize the EMG streamer
        if EMG_DEVICE == 'Quattrocento':
            self.streamer = EMGQCStreamer(MUSCLE_COUNT)
        elif EMG_DEVICE == 'MuoviPro':
            self.streamer = EMGMUOVIStreamer(MUSCLE_COUNT)
        elif EMG_DEVICE == 'Simulation':
            self.path = rospy.get_param(
                "/file_dir") + "/src/mobile_hdemg_exo/new_emg_muovi_data1.csv"
            self.streamer = EMGFileStreamer(
                MUSCLE_COUNT, SAMPLING_FREQUENCY, self.path)
        # Match the streamer's publishing rate
        self.r = rospy.Rate(self.streamer.sample_frequency)
        self.streamer.initialize()

        # Initialize the EMG processor
        self.processor = None
        if EMG_PROCESS_METHOD == 'CST':
            self.processor = EMGProcessorCST()

    def publish_reading(self, publisher: rospy.topics.Publisher, reading: float):
        """
        Publishes the EMG reading to a ROS topic.

        Args:
            publisher: A ROS publisher object.
            reading: A list of integers representing an EMG reading.
        """
        message = hdemg()
        message.header.stamp = rospy.get_rostime()
        message.data = Float64(data=reading)
        publisher.publish(message)

    def pwm_cleanup(self):
        self.p.stop()
        GPIO.cleanup()

    def run_emg(self):
        """
        Runs the EMG streamer and processor.

        Reads EMG data from the streamer, processes it using the selected method, and publishes the raw and processed data
        to ROS topics.
        """
        raw_reading = self.streamer.stream_data()

        # Device-specific processing
        if EMG_DEVICE == 'Quattrocento':
            offset = 32 * MUSCLE_COUNT
            # Each MULTIPLE IN has 64 channels
            hdemg_reading = raw_reading[offset:offset + MUSCLE_COUNT * 64]
        elif EMG_DEVICE == 'MuoviPro':
            # Each Muovi+ EMG probe has 70 channels. Last 6 channels are IMU data
            hdemg_reading = raw_reading[:MUSCLE_COUNT * 64]
            # Publish IMU data
            imu_reading = raw_reading[64:70]
            imu_msg = imu()
            imu_msg.header.stamp = rospy.get_rostime()
            imu_msg.data = Float64MultiArray(data=imu_reading)
            self.imu_pub.publish(imu_msg)
        elif EMG_DEVICE == 'Simulation':
            hdemg_reading = raw_reading  # Simulation data is already in hdemg format
        else:
            raise ValueError('Invalid EMG_DEVICE')

        # Method-specific processing
        if LATENCY_ANALYZER_MODE and EMG_DEVICE == 'Quattrocento':
            processed_emg = raw_reading[96]  # Auxiliary channel 1
        elif LATENCY_ANALYZER_MODE and EMG_DEVICE == 'MuoviPro':
            processed_emg = hdemg_reading[-1]  # Last channel is auxiliary
        elif EMG_PROCESS_METHOD == 'RMS':
            processed_emg = (np.mean(hdemg_reading ** 2))**0.5
            smooth_emg = MovingAverage(100).get_smoothed_value(processed_emg)
            self.publish_reading(self.emg_pub, smooth_emg)
        elif EMG_PROCESS_METHOD == 'CST':
            processed_emg = self.processor.process_reading(hdemg_reading)
            smooth_emg = MovingAverage(100).get_smoothed_value(processed_emg)
            self.processor.publish_reading(self.emg_pub, smooth_emg)
        elif EMG_PROCESS_METHOD == 'Raw':
            processed_emg = np.mean(hdemg_reading)
            self.publish_reading(self.emg_pub, processed_emg)
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
