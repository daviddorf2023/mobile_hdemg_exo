import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import MultiArrayLayout
import math


def publish_emg_data():
    """
    Publishes simulated EMG data to the /hdEMG_Simulation topic. The data is a sin wave with a frequency of 10 Hz and an amplitude of 1.0.
    It is meant to emulate the EMG data from the Muovi+Pro device."""
    # Initialize the ROS node
    rospy.init_node('emg_publisher', anonymous=True)

    # Create a publisher for the /hdEMG_Simulation topic
    pub = rospy.Publisher('/hdEMG_Simulation',
                          Float64MultiArray, queue_size=10)

    # Set the loop rate to 512 Hz
    rate = rospy.Rate(rospy.get_param("/sampling_frequency", int))

    # Initialize the dimensions of the Float64MultiArray
    dim = MultiArrayDimension()
    dim.label = "channels"
    dim.size = 64
    dim.stride = 64

    # Initialize the layout of the Float64MultiArray
    layout = MultiArrayLayout()
    layout.dim.append(dim)
    layout.data_offset = 0

    # Initialize the StampedFloat64MultiArray
    emg_data = Float64MultiArray()

    # Set the amplitude and frequency of the sin wave
    amplitude = 10.0
    frequency = 100.0

    # Set the phase offsets for each channel
    phase_offsets = [math.pi/2, math.pi/4, 0, -math.pi/4, -math.pi/2]

    # Loop until the node is shut down
    while not rospy.is_shutdown():
        # Wait for the /use_simulated_device parameter to be set to True
        while not rospy.get_param("/use_simulated_device"):
            rospy.sleep(0.1)
        data = []
        for i in range(64):
            phase_offset = phase_offsets[i % len(phase_offsets)]
            value = amplitude * \
                math.sin(2 * math.pi * frequency *
                         rospy.get_time() + phase_offset)
            data.append(value)

        # Set the data of the Float64MultiArray
        emg_data.data = data

        # Publish the StampedFloat64MultiArray
        pub.publish(emg_data)

        # Sleep to maintain the loop rate
        rate.sleep()


if __name__ == '__main__':
    try:
        publish_emg_data()
    except rospy.ROSInterruptException:
        pass
