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

# Launch file arguments
EMG_DEVICE = rospy.get_param("/device")
EMG_PROCESS_METHOD = rospy.get_param("/method")
LATENCY_ANALYZER_MODE = rospy.get_param("/latency_analyzer")
MUSCLE_COUNT = rospy.get_param("/muscle_count")
PWM_OUTPUT_PIN = rospy.get_param("/pwm_output_pin")

class EMGStreamNode:
    def __init__(self):
        self.streamer = None  # Stream EMG data from the selected device
        self.raw_pub = rospy.Publisher('hdEMG_stream', hdemg, queue_size=1)
        self.processed_pub = rospy.Publisher('hdEMG_stream_processed', hdemg, queue_size=1)
        if LATENCY_ANALYZER_MODE:
            self.p = GPIO.PWM(PWM_OUTPUT_PIN, 50) # 50 Hz
        if EMG_DEVICE == 'qc':
            self.streamer = EMGQCStreamer(MUSCLE_COUNT)
        elif EMG_DEVICE == 'muovi':
            self.streamer = EMGMUOVIStreamer(MUSCLE_COUNT)
        elif EMG_DEVICE == 'file':
            self.path = rospy.get_param("/file_dir")
            self.path += "/src/talker_listener/raw_emg_34.csv"
            self.streamer = EMGFileStreamer(MUSCLE_COUNT, 512, self.path)
        rospy.init_node('emg_stream_node')
        self.r = rospy.Rate(self.streamer.sample_frequency)  # Match the streamer's publishing rate
        self.streamer.initialize()
        self.processor = None  # Process EMG data using the selected method
        if EMG_PROCESS_METHOD == 'rms':
            self.processor = EMGProcessorRMS()
        elif EMG_PROCESS_METHOD == 'cst':
            self.processor = EMGProcessorCST()
    
    def topic_publish_reading(publisher: rospy.topics.Publisher, reading: "list[int]"):
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

    def pwm_setup(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(PWM_OUTPUT_PIN, GPIO.OUT, initial=GPIO.HIGH)
        self.p.start(50) # 50% duty cycle
    
    def pwm_cleanup(self):
        self.p.stop()
        GPIO.cleanup()

    def run_emg(self):
        print("Running EMG")
        emg_reading = self.streamer.stream_data()
        self.topic_publish_reading(self.raw_pub, emg_reading)
        if LATENCY_ANALYZER_MODE:
            self.pwm_setup()
        if EMG_DEVICE == 'qc': 
            # First MUSCLE_COUNT * 32 channels are for IN1..IN8, two INs per muscle
            offset = 128
            # Each MULTIPLE IN has 64 channels
            hdemg_reading = emg_reading[offset:offset + MUSCLE_COUNT * 64]
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

if __name__ == '__main__':
    emg_stream_node = EMGStreamNode()
    while not rospy.is_shutdown():
        emg_stream_node.run_emg()
    if LATENCY_ANALYZER_MODE:
        emg_stream_node.pwm_cleanup()
    emg_stream_node.close()