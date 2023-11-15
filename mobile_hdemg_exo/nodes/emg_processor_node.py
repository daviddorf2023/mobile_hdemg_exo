import rospy
from std_msgs.msg import Float64MultiArray
from mobile_hdemg_exo.msg import StampedFloat64MultiArray
import numpy as np
from scipy import signal
from mobile_hdemg_exo.processors.emg_process_cst import EMGProcessorCST
from mobile_hdemg_exo.utils.moving_average import MovingAverage


SAMPLING_FREQUENCY = rospy.get_param("/sampling_frequency", int)
REMOVED_CHANNELS = rospy.get_param("/channels_to_remove")
EMG_PROCESS_METHOD = rospy.get_param("/method")
moving_avg_object = MovingAverage(window_size=100)


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


def butter_bandpass(lowcut, highcut, fs, order=5):
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


def csv_output(emg_reading):
    """
    Outputs the EMG reading to a CSV file.

    Args:
        emg_reading: A list of integers representing an EMG reading.
    """
    with open('emg_data.csv', 'a') as f:
        f.write(str(emg_reading) + '\n')


def moving_avg(emg_reading):
    """
    Smoothes the EMG reading.

    Args:
        emg_reading: A list of integers representing an EMG reading.

    Returns:
        A list of integers representing a smoothed EMG reading.
    """
    moving_avg.add_data_point(emg_reading)
    return moving_avg.get_smoothed_value()


def callback(data):
    # Publish processed EMG data
    notch_reading = notch_filter(data)
    b, a = butter_bandpass(20, 100, SAMPLING_FREQUENCY)
    hdemg_filtered = signal.filtfilt(b, a, notch_reading)
    if EMG_PROCESS_METHOD == 'RMS':
        for channel in REMOVED_CHANNELS:
            hdemg_filtered[channel] = np.mean(hdemg_filtered)
        rms_emg = (np.mean(np.array(hdemg_filtered)**2))**0.5
        smooth_emg = moving_avg(rms_emg)
        csv_output(smooth_emg)
    elif EMG_PROCESS_METHOD == 'CST':
        for channel in REMOVED_CHANNELS:
            hdemg_filtered[channel] = 0
        processed_emg = EMGProcessorCST().process_reading(hdemg_filtered/100)
        smooth_emg = moving_avg(processed_emg)
        csv_output(smooth_emg)
    else:
        raise ValueError('Invalid EMG_PROCESS_METHOD')
    pub.publish(smooth_emg)


if __name__ == '__main__':
    rospy.init_node('emg_processor_node')
    sub = rospy.Subscriber(
        'hdEMG_stream_raw', Float64MultiArray, callback)
    pub = rospy.Publisher('hdEMG_stream_processor',
                          StampedFloat64MultiArray, queue_size=10)
    rospy.spin()
