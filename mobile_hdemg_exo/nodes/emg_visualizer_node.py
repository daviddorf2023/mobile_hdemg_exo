import rospy
from std_msgs.msg import Float64MultiArray
from mobile_hdemg_exo.msg import StampedFloat64MultiArray
import numpy as np

while not rospy.get_param("startup_gui_completed"):
    rospy.sleep(0.1)

MUSCLE_COUNT = rospy.get_param("/muscle_count")
SAMPLING_FREQUENCY = rospy.get_param("/sampling_frequency")


class EMGVisualizerNode:
    def __init__(self):
        rospy.init_node('emg_visualizer_node')
        self.r = rospy.Rate(SAMPLING_FREQUENCY)
        self.raw_data = None
        self.start_time = rospy.get_time()
        self.raw_sub = rospy.Subscriber(
            'hdEMG_stream_raw', StampedFloat64MultiArray, self.callback)
        self.visualizer_pub = rospy.Publisher(
            'hdEMG_stream_visual', StampedFloat64MultiArray, queue_size=10)

    def callback(self, raw_message):
        self.raw_data = raw_message.data.data

    def visualize_emg(self, raw_message):
        """
        Normalizes, adds spacing, and removes channels from the raw EMG data for visualization.
        """
        removed_channels = rospy.get_param("/channels_to_remove")
        if removed_channels != '':
            removed_channels = removed_channels.split(',')
            removed_channels = list(map(int, removed_channels))
            removed_channels = [
                x for x in removed_channels if x < MUSCLE_COUNT * 64]
        else:
            removed_channels = []
        normalization_factor = np.max(np.abs(self.raw_data))
        visual_emg = self.raw_data + \
            normalization_factor * np.arange(len(self.raw_data))
        visual_emg = visual_emg / \
            normalization_factor
        visual_emg = np.delete(
            visual_emg, removed_channels)
        visual_message = StampedFloat64MultiArray()
        visual_message.header.stamp = rospy.Time.now()
        visual_message.data = Float64MultiArray(data=visual_emg)
        self.visualizer_pub.publish(visual_message)


if __name__ == '__main__':
    emg_visualizer_node = EMGVisualizerNode()
    while not rospy.is_shutdown():
        if emg_visualizer_node.raw_data is not None:
            emg_visualizer_node.visualize_emg(emg_visualizer_node.raw_data)
        emg_visualizer_node.r.sleep()
