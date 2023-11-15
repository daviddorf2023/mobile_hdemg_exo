import rospy
from std_msgs.msg import Float64MultiArray
from mobile_hdemg_exo.msg import StampedFloat64MultiArray
from std_msgs.msg import Float64MultiArray
import numpy as np

MUSCLE_COUNT = rospy.get_param("/muscle_count")


def callback(data):
    removed_channels = rospy.get_param("/channels_to_remove")
    if removed_channels != '':
        removed_channels = removed_channels.split(',')
        removed_channels = list(map(int, removed_channels))
        removed_channels = [
            x for x in removed_channels if x < MUSCLE_COUNT * 64]
    else:
        removed_channels = []
    hdemg_reading = data.data
    normalization_factor = np.max(np.abs(hdemg_reading))
    visual_emg = hdemg_reading + \
        normalization_factor * np.arange(len(hdemg_reading))
    visual_emg = visual_emg / \
        normalization_factor
    visual_emg = np.delete(
        visual_emg, removed_channels)
    visual_message = StampedFloat64MultiArray()
    visual_message.header.stamp = rospy.get_rostime().from_sec(
        rospy.get_time() - start_time)
    visual_message.data = Float64MultiArray(data=visual_emg)
    pub.publish(visual_message)


if __name__ == '__main__':
    rospy.init_node('emg_visualizer_node')
    start_time = rospy.get_time()
    sub = rospy.Subscriber(
        'hdEMG_stream_raw', Float64MultiArray, callback)
    pub = rospy.Publisher('hdEMG_stream_visual',
                          StampedFloat64MultiArray, queue_size=10)
    rospy.spin()
