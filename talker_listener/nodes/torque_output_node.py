import rospy
from h3_msgs.msg import State
from talker_listener.msg import hdemg

while not rospy.get_param("calibrated"):
    rospy.sleep(0.1)

def main():
    """
    Publishes torque commands from EMG input to the /h3/robot_states topic.
    """
    rospy.init_node('torque_stream')
    r = rospy.Rate(100)
    emg_coef = rospy.get_param("emg_coef")
    emg_sub = rospy.Subscriber('/hdEMG_stream_processed', hdemg)
    torque_pub = rospy.Publisher('/h3/robot_states', State, queue_size=10)

    while not rospy.is_shutdown():
        emg_data = emg_sub.data.data
        sample = State()
        sample.header.stamp = rospy.Time.now()
        sample.joint_torque_sensor, sample.joint_position, sample.joint_velocity, sample.joint_torque_sensor = [0,0,0,0]
        sample.joint_motor_torque = emg_coef * emg_data
        torque_pub.publish(sample)
        r.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass