#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from talker_listener.msg import hdemg
from talker_listener.processors.emg_process_rms import EMGProcessorRMS
from talker_listener.processors.emg_process_cst import EMGProcessorCST
from talker_listener.streamer.emg_file_streamer import EMGFileStreamer
from talker_listener.streamer.emg_qc_streamer import EMGQCStreamer
from talker_listener.streamer.emg_muovi_streamer import EMGMUOVIStreamer
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
        self.streamer = None
        self.processed_pub = rospy.Publisher('hdEMG_stream_processed', hdemg, queue_size=1)

        # Initialize the PWM output pin
        if LATENCY_ANALYZER_MODE:
            self.start_time = rospy.get_time()
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
            self.path = rospy.get_param("/file_dir") + "/src/talker_listener/raw_emg_34.csv"
            self.streamer = EMGFileStreamer(MUSCLE_COUNT, SAMPLING_FREQUENCY, self.path)
        self.r = rospy.Rate(self.streamer.sample_frequency)  # Match the streamer's publishing rate
        self.streamer.initialize()

        # Initialize the EMG processor
        self.processor = None
        if EMG_PROCESS_METHOD == 'RMS':
            self.processor = EMGProcessorRMS()
        elif EMG_PROCESS_METHOD == 'CST':
            self.processor = EMGProcessorCST()

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
        # TODO: Reestablish file streaming [depends on the device used to collect data]
        if LATENCY_ANALYZER_MODE:
            hdemg_reading = raw_reading[96]
            processed_reading = hdemg_reading # No processing needed in latency analyzer mode (tests EMG device latency), rostopic delay measures processing time
        else:
            processed_reading = self.processor.process_reading(hdemg_reading)
        self.publish_reading(self.processed_pub, processed_reading)
        self.r.sleep()


if __name__ == '__main__':
    emg_stream_node = EMGStreamNode()
    while not rospy.is_shutdown() and rospy.get_time() - emg_stream_node.start_time < TRIAL_DURATION_SECONDS:
        emg_stream_node.run_emg()
    emg_stream_node.pwm_cleanup()
    emg_stream_node.streamer.close()
