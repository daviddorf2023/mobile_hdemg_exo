#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Float64MultiArray


def plot_array(data):
    plt.clf()
    for i in range(len(data.data)):
        plt.subplot(64, 1, i+1)
        plt.plot(data.data[i])
    plt.draw()
    plt.pause(0.001)


if __name__ == '__main__':
    rospy.init_node('realtime_plotter')
    rospy.Subscriber('/hdEMG_stream_raw', Float64MultiArray, plot_array)
    plt.ion()
    plt.show()
    rospy.spin()
