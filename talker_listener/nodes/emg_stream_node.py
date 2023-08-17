#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from talker_listener.msg import hdemg
from talker_listener.processors.emg_process_rms import EMGProcessorRMS
from talker_listener.processors.emg_process_cst import EMGProcessorCST
from talker_listener.streamer.emg_file_streamer import EMGFileStreamer
from talker_listener.streamer.emg_qc_streamer import EMGQCStreamer
from talker_listener.streamer.emg_muovi_streamer import EMGMUOVIStreamer
import RPi.GPIO as GPIO
import time

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
        self.streamer = None  # Stream EMG data from the selected device
        self.raw_pub = rospy.Publisher('hdEMG_stream', hdemg, queue_size=1)
        self.processed_pub = rospy.Publisher('hdEMG_stream_processed', hdemg, queue_size=1)
        self.raw_muscle_reading = np.zeros((SAMPLING_FREQUENCY*TRIAL_DURATION_SECONDS, MUSCLE_COUNT*64), dtype=np.int16)  # Initialize the raw EMG reading array

        # Initialize the PWM output pin
        if LATENCY_ANALYZER_MODE:
            self.start_time = time.time()  # TODO: Use rospy.get_rostime() instead?
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(PWM_OUTPUT_PIN, GPIO.OUT, initial=GPIO.HIGH)
            self.p = GPIO.PWM(PWM_OUTPUT_PIN, 50) # 50 Hz
            self.p.start(50) # 50% duty cycle
            self.pwm_time = (time.time() - self.start_time)
            print("PWM started " + str(self.pwm_time) + " seconds after start")
        
        # Initialize the EMG streamer
        if EMG_DEVICE == 'Quattrocento':
            self.streamer = EMGQCStreamer(MUSCLE_COUNT)
        elif EMG_DEVICE == 'MuoviPro':
            self.streamer = EMGMUOVIStreamer(MUSCLE_COUNT)
        elif EMG_DEVICE == 'Simulation':
            self.path = rospy.get_param("/file_dir") + "/src/talker_listener/raw_emg_34.csv"
            self.streamer = EMGFileStreamer(MUSCLE_COUNT, SAMPLING_FREQUENCY, self.path)
        rospy.init_node('emg_stream_node')
        self.r = rospy.Rate(self.streamer.sample_frequency)  # Match the streamer's publishing rate
        self.streamer.initialize()
        if LATENCY_ANALYZER_MODE:
            self.streamer_time = (time.time() - self.start_time)

        # Initialize the EMG processor
        self.processor = None  # Process EMG data using the selected method
        if EMG_PROCESS_METHOD == 'RMS':
            self.processor = EMGProcessorRMS()
        elif EMG_PROCESS_METHOD == 'CST':
            self.processor = EMGProcessorCST()

    def topic_publish_reading(self, publisher: rospy.topics.Publisher, reading: "list[np.int16]"):
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
        time_counter=0

        if EMG_DEVICE == 'Quattrocento':
            offset = 32 * MUSCLE_COUNT
            hdemg_reading = emg_reading[offset:offset + MUSCLE_COUNT * 64]  # Each MULTIPLE IN has 64 channels
            for i in range(MUSCLE_COUNT):
                # Replace each row of the raw_muscle_reading array with the reading from each muscle
                self.raw_muscle_reading[time_counter, i * 64:(i + 1) * 64] = hdemg_reading[i * 64:(i + 1) * 64]

        elif EMG_DEVICE == 'MuoviPro': 
            hdemg_reading = emg_reading[:64]  # Each Muovi+ probe has 70 channels. Keep only first 64 channels, last 6 are IMU data
            for i in range(MUSCLE_COUNT):
                self.raw_muscle_reading[time_counter, i * 64:(i + 1) * 64] = hdemg_reading[i * 64:(i + 1) * 64]

        time_counter+=1
        self.processor.process_reading(self.raw_muscle_reading)
        self.processor.publish_reading(self.processed_pub)
        self.r.sleep()


if __name__ == '__main__':
    emg_stream_node = EMGStreamNode()
    try:
        while not rospy.is_shutdown() and time.time() - emg_stream_node.start_time < TRIAL_DURATION_SECONDS:
            emg_stream_node.run_emg()
    except rospy.ROSInterruptException:
        pass
    raw_muscle_numpy = np.array(emg_stream_node.raw_muscle_reading)
    np.savetxt("/home/sralexo/Downloads/exo/src/technaid_h3_ankle_ros_python/talker_listener/src/talker_listener/raw_emg.csv", raw_muscle_numpy, delimiter=",")
    if LATENCY_ANALYZER_MODE:
        pwm_output_numpy = raw_muscle_numpy[:, 96*MUSCLE_COUNT]  # Save 96th channel of each muscle
        np.savetxt("/home/sralexo/Downloads/exo/src/technaid_h3_ankle_ros_python/talker_listener/src/talker_listener/pwm_output.csv", pwm_output_numpy, delimiter=",")
    emg_stream_node.streamer.close()
    emg_stream_node.pwm_cleanup()
