#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from mobile_hdemg_exo.msg import StampedFloat64MultiArray


while not rospy.get_param("gui_completed"):
    rospy.sleep(0.1)

EMG_DEVICE = rospy.get_param("/device")
EMG_PROCESS_METHOD = rospy.get_param("/method")
MUSCLE_COUNT = rospy.get_param("/muscle_count", int)
SAMPLING_FREQUENCY = rospy.get_param("/sampling_frequency", int)
LATENCY_ANALYZER_MODE = rospy.get_param("/latency_analyzer")
PWM_OUTPUT_PIN = rospy.get_param("/pwm_output_pin", int)


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
        self.r = rospy.Rate(SAMPLING_FREQUENCY)
        self.start_time = rospy.get_time()
        self.emg_pub = rospy.Publisher(
            'hdEMG_stream_raw', StampedFloat64MultiArray, queue_size=10)

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
        self.emg_offset = 0
        if EMG_DEVICE == 'Quattrocento':
            from mobile_hdemg_exo.streamer.emg_qc_streamer import EMGQCStreamer
            self.streamer = EMGQCStreamer(MUSCLE_COUNT)
            self.emg_offset = 32 * MUSCLE_COUNT
        elif EMG_DEVICE == 'Muovi+Pro':
            from mobile_hdemg_exo.streamer.emg_muovi_streamer import EMGMUOVIStreamer
            self.streamer = EMGMUOVIStreamer(MUSCLE_COUNT)
        elif EMG_DEVICE == 'File':
            from mobile_hdemg_exo.streamer.emg_file_streamer import EMGFileStreamer
            self.path = rospy.get_param(
                "/file_dir") + "/data/cst_test_data_nov1/raw_emg_7.csv"
            self.streamer = EMGFileStreamer(
                MUSCLE_COUNT, SAMPLING_FREQUENCY, self.path)
            self.emg_offset = 128
        else:
            raise ValueError('Invalid EMG_DEVICE')

        self.simulation_reading = np.zeros(64 * MUSCLE_COUNT)

    def simulation_callback(self, data):
        """
        Callback function for the /hdEMG_Simulation topic.

        Args:
            data: A list of floats representing EMG data.
        """
        self.simulation_reading = data.data

    def pwm_cleanup(self):
        self.p.stop()
        GPIO.cleanup()

    def run_emg(self):
        """
        Reads EMG data from the streamer and publishes it to /hdEMG_stream_raw
        """
        raw_reading = self.streamer.stream_data()
        hdemg_reading = raw_reading[self.emg_offset:
                                    self.emg_offset + 64 * MUSCLE_COUNT]
        raw_message = StampedFloat64MultiArray()
        raw_message.header.stamp = rospy.get_rostime().from_sec(
            rospy.get_time() - self.start_time)
        raw_message.data = Float64MultiArray(data=hdemg_reading)
        self.emg_pub.publish(raw_message)
        self.r.sleep()


if __name__ == '__main__':
    emg_stream_node = EMGStreamNode()
    while not rospy.is_shutdown():
        emg_stream_node.run_emg()
    if LATENCY_ANALYZER_MODE:
        emg_stream_node.pwm_cleanup()
