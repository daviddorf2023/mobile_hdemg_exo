import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from talker_listener.msg import hdemg

from talker_listener.moving_window import MovingWindow


class EMGProcessorRMS:
    def __init__(self):
        self.window = MovingWindow(100)
        self.data = []

    def process_reading(self, reading):
        self.window.append(reading)

    @staticmethod
    def rms_muscle_reading_window(window: MovingWindow) -> np.ndarray:
        """
        Calculates RMS emg across time for each channel and average 64 channels together for each muscle.
        """
        # RMS of each channel in each muscle across the time window
        smoothed_rms_muscle_reading = np.sqrt(np.mean(np.square(window), axis=0))

        # Average of the 64 RMS'ed channels in each muscle
        avg_muscle_reading = np.mean(smoothed_rms_muscle_reading, axis=1)

        return avg_muscle_reading

    def publish_reading(self, publisher: rospy.Publisher):
        # reading.shape -> (3) // MUSCLE_COUNT
        reading = self.rms_muscle_reading_window(self.window)
        self.data.append(reading)

        stamped_sample = hdemg()
        stamped_sample.header.stamp = rospy.get_rostime()

        sample = Float64MultiArray()
        sample.data = reading
        sample.layout.dim = [
            MultiArrayDimension("rows", 1, len(reading)),
        ]

        stamped_sample.data = sample
        publisher.publish(stamped_sample)
