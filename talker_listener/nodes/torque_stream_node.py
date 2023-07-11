#!/usr/bin/env python3

import rospy
from h3_msgs.msg import State

from talker_listener.streamer.torque_file_streamer import TorqueFileStreamer


def topic_publish_reading(publisher: rospy.topics.Publisher, reading):
    sample = State()
    sample.header.stamp = rospy.Time.now()
    sample.joint_torque_sensor = reading
    sample.joint_position = reading
    sample.joint_velocity = reading
    sample.joint_motor_torque = reading
    sample.joint_torque_sensor = reading

    publisher.publish(sample)


if __name__ == '__main__':
    rospy.init_node('torque_stream_node')
    pub = rospy.Publisher('/h3/robot_states', State, queue_size=10)

    path = rospy.get_param("/file_dir")
    path += "/src/talker_listener/raw_torque_34.csv"

    streamer = TorqueFileStreamer(512, path)
    streamer.initialize()
    r = rospy.Rate(streamer.sample_frequency)  # Match the streamer's publishing rate
    while not rospy.is_shutdown():
        torque_reading = streamer.stream_data()
        topic_publish_reading(pub, torque_reading)
        r.sleep()
