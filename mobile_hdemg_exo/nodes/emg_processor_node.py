import rospy
from std_msgs.msg import Float64
from mobile_hdemg_exo.msg import StampedFloat64MultiArray, StampedFloat64
import numpy as np
from scipy import signal
from mobile_hdemg_exo.processors.emg_process_cst import EMGProcessorCST

while not rospy.get_param("startup_gui_completed"):
    rospy.sleep(0.1)

SAMPLING_FREQUENCY = rospy.get_param("/sampling_frequency", int)
REMOVED_CHANNELS = rospy.get_param("/channels_to_remove")
EMG_PROCESS_METHOD = rospy.get_param("/method")


class EMGProcessorNode:
    """
    A class for processing EMG data.

    Attributes:
        r: A ROS Rate object.
        raw_sub: A ROS subscriber for the /hdEMG_stream_raw topic.
        processed_pub: A ROS publisher for the /hdEMG_stream_processed topic.
        raw_data: A list of integers representing EMG data.
        raw_timestamp: The timestamp of the EMG data.
        moving_avg_object: A MovingAverage object.
        start_time: The time at which the node was started.
    """

    def __init__(self):
        rospy.init_node('emg_processor_node')
        self.r = rospy.Rate(SAMPLING_FREQUENCY)
        self.raw_sub = rospy.Subscriber(
            'hdEMG_stream_raw', StampedFloat64MultiArray, self.callback)
        self.processed_pub = rospy.Publisher(
            'hdEMG_stream_processed', StampedFloat64, queue_size=10)
        if EMG_PROCESS_METHOD == 'CST':
            self.processor = EMGProcessorCST()
        self.raw_data = None
        self.raw_timestamp = None
        self.start_time = rospy.get_time()
        self.smoothing_window = []
        self.smoothing_window_size = SAMPLING_FREQUENCY  # Default to 1 second
        self.b, self.a = self.butter_bandpass(20, 100, SAMPLING_FREQUENCY)

    @staticmethod
    def notch_filter(data):
        """ 
        Applies a notch filter to the EMG data.

        Args:
            data: A list of integers representing an EMG reading.

        Returns:
            A list of integers representing an EMG reading with a notch filter applied.
        """
        b, a = signal.iirnotch(60, 30, SAMPLING_FREQUENCY)
        return signal.filtfilt(b, a, data)

    @staticmethod
    def butter_bandpass(lowcut, highcut, fs, order=2):
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

    @staticmethod
    def csv_output(emg_reading):
        """
        Outputs the EMG reading to a CSV file.

        Args:
            emg_reading: A list of integers representing an EMG reading.
        """
        with open('emg_data.csv', 'a') as f:
            f.write(str(emg_reading) + '\n')

    def moving_avg(self, data):
        """
        Calculates the moving average of the EMG data.

        Args:
            data: A list of integers representing an EMG reading.

        Returns:
            A list of integers representing an EMG reading with a moving average applied.
        """
        self.smoothing_window.append(data)
        if len(self.smoothing_window) > self.smoothing_window_size:
            self.smoothing_window.pop(0)
        return np.mean(self.smoothing_window)

    def callback(self, raw_message):
        self.raw_data = raw_message.data.data

    def process_emg(self):
        notch_reading = self.notch_filter(self.raw_data)
        # hdemg_filtered = signal.filtfilt(self.b, self.a, notch_reading)
        hdemg_filtered = notch_reading
        if EMG_PROCESS_METHOD == 'RMS':
            for channel in REMOVED_CHANNELS:
                hdemg_filtered[channel] = np.mean(hdemg_filtered)
            rms_emg = (np.mean(np.array(hdemg_filtered)**2))**0.5
            smooth_emg = self.moving_avg(rms_emg)
            self.csv_output(smooth_emg)
        elif EMG_PROCESS_METHOD == 'CST':
            for channel in REMOVED_CHANNELS:
                hdemg_filtered[channel] = 0
            processed_emg = self.processor.process_reading(hdemg_filtered/10)
            if processed_emg:
                processed_emg = 10 * processed_emg[0]
            else:
                processed_emg = 0
            smooth_emg = self.moving_avg(processed_emg)
            self.csv_output(smooth_emg)
        else:
            raise ValueError('Invalid EMG_PROCESS_METHOD')
        processor_message = StampedFloat64()
        processor_message.header.stamp = rospy.get_rostime().from_sec(
            rospy.get_time() - self.start_time)
        processor_message.data = Float64(data=smooth_emg)
        self.processed_pub.publish(processor_message)


if __name__ == '__main__':
    emg_processor_node = EMGProcessorNode()
    while not rospy.is_shutdown():
        if emg_processor_node.raw_data is not None:
            emg_processor_node.process_emg()
        emg_processor_node.r.sleep()
