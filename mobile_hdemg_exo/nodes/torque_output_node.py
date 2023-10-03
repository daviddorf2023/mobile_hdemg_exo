import rospy
from std_msgs.msg import Float64
from h3_msgs.msg import State
from mobile_hdemg_exo.msg import hdemg
from mobile_hdemg_exo.streamer.emg_qc_streamer import EMGQCStreamer
from mobile_hdemg_exo.streamer.emg_muovi_streamer import EMGMUOVIStreamer
from mobile_hdemg_exo.utils.moving_average import MovingAverage

while not rospy.get_param("calibrated"):
    rospy.sleep(0.1)

EMG_DEVICE = rospy.get_param("/device")
MUSCLE_COUNT = rospy.get_param("/muscle_count", int)
SAMPLING_FREQUENCY = rospy.get_param("/sampling_frequency", int)


class TorqueOutputNode:

    def __init__(self):
        if (rospy.get_param("/side") == "Left"):
            self.torque_pub = rospy.Publisher(
                '/h3/left_ankle_effort_controller/command', Float64, queue_size=10)
        elif (rospy.get_param("/side") == "Right"):
            self.torque_pub = rospy.Publisher(
                '/h3/right_ankle_effort_controller/command', Float64, queue_size=10)
        if EMG_DEVICE == 'Quattrocento':
            self.streamer = EMGQCStreamer(MUSCLE_COUNT)
        elif EMG_DEVICE == 'MuoviPro':
            self.streamer = EMGMUOVIStreamer(MUSCLE_COUNT)
        self.emg_sub = rospy.Subscriber(
            '/hdEMG_stream_processed', hdemg, self.emg_callback)
        self.torque_sensor_sub = rospy.Subscriber(
            '/h3/robot_states', State, self.sensor_callback)
        self.emg_data = 0
        self.old_torque = 0
        self.emg_coef = rospy.get_param("/emg_coef")

    def sensor_callback(self, sensor_reading):
        ''' Callback for /h3/robot_states. Reads sensor messages from the h3 and saves them in class variables.
        '''
        self.sensor_torque = sensor_reading.joint_torque_sensor[2]

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

        # For having foot in the exo
        if self.sensor_torque < -1:
            self.torque_command = self.emg_coef_down * self.emg_data
        elif self.sensor_torque > 1:
            self.torque_command = self.emg_coef_up * self.emg_data
        else:
            self.torque_command = 0
        smooth_torque_command = MovingAverage(100).get_smoothed_value(
            self.torque_command)
        self.torque_pub.publish(smooth_torque_command)
        r.sleep()


if __name__ == '__main__':
    torque_output_node = TorqueOutputNode()
    while not rospy.is_shutdown():
        torque_output_node.torque_output()
