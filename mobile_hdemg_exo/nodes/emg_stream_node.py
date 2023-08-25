#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from mobile_hdemg_exo.msg import hdemg
from mobile_hdemg_exo.processors.emg_process_cst import EMGProcessorCST
from mobile_hdemg_exo.streamer.emg_file_streamer import EMGFileStreamer
from mobile_hdemg_exo.streamer.emg_qc_streamer import EMGQCStreamer
from mobile_hdemg_exo.streamer.emg_muovi_streamer import EMGMUOVIStreamer
import RPi.GPIO as GPIO

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
        self.processed_pub = rospy.Publisher('hdEMG_stream_processed', hdemg, queue_size=1)
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
            self.path = rospy.get_param("/file_dir") + "/src/mobile_hdemg_exo/new_emg_muovi_data1.csv"
            self.streamer = EMGFileStreamer(MUSCLE_COUNT, SAMPLING_FREQUENCY, self.path)
        self.r = rospy.Rate(self.streamer.sample_frequency)  # Match the streamer's publishing rate
        self.streamer.initialize()

        # Initialize the EMG processor
        self.processor = None
        if EMG_PROCESS_METHOD == 'CST':
            self.processor = EMGProcessorCST()

    def smoothed_rms(self, hdemg_reading):
        """
        Calculates RMS emg.

        Args:
            hdemg_reading: A list of integers representing an EMG reading.

        Returns:
            A float representing the RMS EMG reading.
        """
        sum_squares = sum(x**2 for x in hdemg_reading)
        rms_muscle_reading = (sum_squares / len(hdemg_reading)) ** 0.5
        return rms_muscle_reading

    def publish_reading(self, publisher: rospy.topics.Publisher, reading: float):
        """
        Publishes the EMG reading to a ROS topic.

        Args:
            publisher: A ROS publisher object.
            reading: A list of integers representing an EMG reading.
        """
        message = hdemg()
        message.header.stamp = rospy.Time.now()
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
        if EMG_DEVICE == 'Quattrocento':
            offset = 32 * MUSCLE_COUNT
            hdemg_reading = raw_reading[offset:offset + MUSCLE_COUNT * 64]  # Each MULTIPLE IN has 64 channels
        elif EMG_DEVICE == 'MuoviPro': 
            hdemg_reading = raw_reading[:MUSCLE_COUNT * 64]  # Each Muovi+ probe has 70 channels. Keep only first 64 channels, last 6 are IMU data
        else:
            hdemg_reading = raw_reading  # Simulation data is already in hdemg format

        if LATENCY_ANALYZER_MODE and EMG_DEVICE == 'Quattrocento':
            processed_reading = raw_reading[96]
        elif LATENCY_ANALYZER_MODE and EMG_DEVICE == 'MuoviPro':
            processed_reading = hdemg_reading[-1]
        elif EMG_PROCESS_METHOD == 'RMS':
            rms_reading = self.smoothed_rms(hdemg_reading)
            processed_reading = (rms_reading + self.old_reading) / 2  # Low-pass filter
            self.old_reading = processed_reading
        else:
            processed_reading = self.processor.process_reading(hdemg_reading)
        self.publish_reading(self.processed_pub, processed_reading)
        self.r.sleep()


if __name__ == '__main__':
    emg_stream_node = EMGStreamNode()
    while not rospy.is_shutdown():
        emg_stream_node.run_emg()
    if LATENCY_ANALYZER_MODE:
        emg_stream_node.pwm_cleanup()
    emg_stream_node.streamer.close()

