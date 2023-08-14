#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from talker_listener.msg import hdemg
from talker_listener.processors.emg_process_rms import EMGProcessorRMS
from talker_listener.processors.emg_process_cst import EMGProcessorCST
from talker_listener.streamer.emg_file_streamer import EMGFileStreamer
from talker_listener.streamer.emg_qc_streamer import EMGQCStreamer
from talker_listener.streamer.emg_muovi_streamer import EMGMUOVIStreamer
import RPi.GPIO as GPIO
import time

# Launch file arguments
EMG_DEVICE = rospy.get_param("/device")
EMG_PROCESS_METHOD = rospy.get_param("/method")
LATENCY_ANALYZER_MODE = rospy.get_param("/latency_analyzer")
MUSCLE_COUNT = rospy.get_param("/muscle_count")
PWM_OUTPUT_PIN = rospy.get_param("/pwm_output_pin")
SAMPLING_FREQUENCY = rospy.get_param("/sampling_frequency")

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
        self.streamer = None  # Stream EMG data from the selected device
        self.raw_pub = rospy.Publisher('hdEMG_stream', hdemg, queue_size=1)
        self.processed_pub = rospy.Publisher('hdEMG_stream_processed', hdemg, queue_size=1)

        # Initialize the PWM output pin
        if LATENCY_ANALYZER_MODE:
            self.start_time = time.time()
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(PWM_OUTPUT_PIN, GPIO.OUT, initial=GPIO.HIGH)
            self.p = GPIO.PWM(PWM_OUTPUT_PIN, 1) # 1 Hz
            self.p.start(50) # 50% duty cycle
            self.pwm_time = (time.time() - self.start_time)
            print("PWM started" + str(self.pwm_start_time))
        
        # Initialize the EMG streamer
        if EMG_DEVICE == 'qc':
            self.streamer = EMGQCStreamer(MUSCLE_COUNT)
        elif EMG_DEVICE == 'muovi':
            self.streamer = EMGMUOVIStreamer(MUSCLE_COUNT)
        elif EMG_DEVICE == 'file':
            self.path = rospy.get_param("/file_dir") + "/src/talker_listener/raw_emg_34.csv"
            self.streamer = EMGFileStreamer(MUSCLE_COUNT, SAMPLING_FREQUENCY, self.path)
        rospy.init_node('emg_stream_node')
        self.r = rospy.Rate(self.streamer.sample_frequency)  # Match the streamer's publishing rate
        self.streamer.initialize()
        if LATENCY_ANALYZER_MODE:
            self.streamer_time = (time.time() - self.start_time)

        # Initialize the EMG processor
        self.processor = None  # Process EMG data using the selected method
        if EMG_PROCESS_METHOD == 'rms':
            self.processor = EMGProcessorRMS()
        elif EMG_PROCESS_METHOD == 'cst':
            self.processor = EMGProcessorCST()

    def topic_publish_reading(self, publisher: rospy.topics.Publisher, reading: "list[int]"):
        """
        Publishes an EMG reading to a ROS topic.

        Args:
            publisher: A ROS publisher object.
            reading: A list of integers representing an EMG reading.
        """
        stamped_sample = hdemg()
        stamped_sample.header.stamp = rospy.get_rostime()
        sample = Float64MultiArray()
        sample.data = reading
        sample.layout.dim = [
            MultiArrayDimension("rows", 1, len(reading)),
            MultiArrayDimension("columns", 1, 1)
        ]
        stamped_sample.data = sample
        publisher.publish(stamped_sample)

    def pwm_cleanup(self):
        self.p.stop()
        GPIO.cleanup()

    def run_emg(self):
        """
        Runs the EMG streamer and processor.

        Reads EMG data from the streamer, processes it using the selected method, and publishes the raw and processed data
        to ROS topics.
        """
        emg_reading = self.streamer.stream_data()
        self.topic_publish_reading(self.raw_pub, emg_reading)
        if EMG_DEVICE == 'qc':
            # The first 128 channels are from the auxiliary ports on the back of the Quattrocento
            if LATENCY_ANALYZER_MODE:
                offset = 0
            else:
                offset = 128
            hdemg_reading = emg_reading[offset:offset + MUSCLE_COUNT * 64]  # Each MULTIPLE IN has 64 channels
            raw_muscle_reading = []
            for i in range(MUSCLE_COUNT):
                raw_muscle_reading.append(hdemg_reading[i * 64:(i + 1) * 64])
        elif EMG_DEVICE == 'muovi': 
            # Each Muovi+ probe has 70 channels
            raw_muscle_reading = []
            # Keep only first 64 channels, last 6 are IMU data
            hdemg_reading = emg_reading[:64]
            for i in range(MUSCLE_COUNT):
                raw_muscle_reading.append(emg_reading)

        self.processor.process_reading(raw_muscle_reading)
        self.processor.publish_reading(self.processed_pub)
        self.r.sleep()

    def close(self):
        self.streamer.close()

if __name__ == '__main__':
    emg_stream_node = EMGStreamNode()
    while not rospy.is_shutdown():
        emg_stream_node.run_emg()
    if LATENCY_ANALYZER_MODE:
        emg_stream_node.pwm_cleanup()
    emg_stream_node.close()