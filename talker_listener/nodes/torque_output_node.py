import rospy
from std_msgs.msg import Float64
from talker_listener.msg import hdemg
from talker_listener.streamer.emg_qc_streamer import EMGQCStreamer
from talker_listener.streamer.emg_muovi_streamer import EMGMUOVIStreamer

while not rospy.get_param("calibrated"):
    rospy.sleep(0.1)

EMG_DEVICE = rospy.get_param("/device")
MUSCLE_COUNT = rospy.get_param("/muscle_count", int)
SAMPLING_FREQUENCY = rospy.get_param("/sampling_frequency", int)

class TorqueOutputNode:

    def __init__(self):
        if EMG_DEVICE == 'Quattrocento':
            self.streamer = EMGQCStreamer(MUSCLE_COUNT)
        elif EMG_DEVICE == 'MuoviPro':
            self.streamer = EMGMUOVIStreamer(MUSCLE_COUNT)
        self.emg_sub = rospy.Subscriber('/hdEMG_stream_processed', hdemg, self.emg_callback)
        self.emg_data = 0
        self.emg_coef = rospy.get_param("emg_coef")

    def emg_callback(self, hdEMG):
        """
        Callback function for the EMG data subscriber.
        """
        self.emg_data = hdEMG.data.data

    def torque_output(self):
        """
        Publishes torque commands from EMG input to the /h3/ankle_effort_controller/command topic.
        """
        rospy.init_node('torque_stream')
        r = rospy.Rate(100)

        if (rospy.get_param("/side") == "Left"):
            torque_pub = rospy.Publisher('/h3/left_ankle_effort_controller/command', Float64, queue_size=10)
        elif (rospy.get_param("/side") == "Right"):
            torque_pub = rospy.Publisher('/h3/right_ankle_effort_controller/command', Float64, queue_size=10)

        while not rospy.is_shutdown():
            torque_command = self.emg_coef * self.emg_data
            print("Publishing torque: " + str(torque_command))
            torque_pub.publish(torque_command)
            r.sleep()
        
        

if __name__ == '__main__':
    torque_output_node = TorqueOutputNode()
    while not rospy.is_shutdown():
        torque_output_node.torque_output()