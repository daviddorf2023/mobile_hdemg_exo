#!/usr/bin/env python
import os

import numpy as np
import pandas as pd
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from talker_listener.msg import hdemg

import talker_listener.qc_communication as comm
from talker_listener.emg_process_cst import EMGProcessorCST
from talker_listener.emg_process_rms import EMGProcessorRMS
from talker_listener.qc_connect_config import NumChanVal

# number of bytes in sample (2 bytes for the quattrocento device)
NBYTES = 2

# refresh rate for quattrocento device (set in OT BioLab Lite)
REFRESH_RATE = 1 / 32

QC_SAMPLING_FREQUENCY = 512
NODE_SAMPLING_FREQUENCY = 512
MUSCLE_COUNT = 4

# EMG processing methods
EMG_PROCESS_RMS = "rms"
EMG_PROCESS_CST = "cst"

EMG_PROCESS_METHOD = "rms"


def process_socket_data(q_socket, muscle_count):
    nchan = NumChanVal[muscle_count - 1]

    # read raw data from socket
    sbytes = comm.read_raw_bytes(
        q_socket,
        nchan,
        NBYTES)

    # convert the bytes into integer values
    sample_from_channels = comm.bytes_to_integers(
        sbytes,
        nchan,
        NBYTES,
        output_milli_volts=False)

    return sample_from_channels


def topic_publish_reading(publisher: rospy.topics.Publisher, reading: list[int]):
    stamped_sample = hdemg()
    stamped_sample.header.stamp = rospy.get_rostime()

    sample = Float64MultiArray()
    sample.data = reading
    sample.layout.dim = [
        # MultiArrayDimension("rows", 1, 6 * 64),
        MultiArrayDimension("rows", 1, len(reading)),
        MultiArrayDimension("columns", 1, 1)
    ]

    stamped_sample.data = sample
    publisher.publish(stamped_sample)


if __name__ == '__main__':
    rospy.init_node('QC_stream_node')
    r = rospy.Rate(QC_SAMPLING_FREQUENCY)  # Match the Quattrocento publishing rate
    raw_pub = rospy.Publisher('hdEMG_stream', hdemg, queue_size=1)
    processed_pub = rospy.Publisher('hdEMG_stream_processed', hdemg, queue_size=1)

    q_socket = comm.connect(REFRESH_RATE, QC_SAMPLING_FREQUENCY, MUSCLE_COUNT)

    timer = rospy.get_time()
    processor = None
    if EMG_PROCESS_METHOD == EMG_PROCESS_RMS:
        processor = EMGProcessorRMS()
    elif EMG_PROCESS_METHOD == EMG_PROCESS_CST:
        processor = EMGProcessorCST()

    data = []
    sample_count = 0
    while not rospy.is_shutdown():
        # emg_reading is an array of 408 ints (408 channels = (8*16) + (4*64) + 16 + 8)
        emg_reading = process_socket_data(q_socket, MUSCLE_COUNT)
        sample_count += 1

        # QC_SAMPLING_FREQUENCY is fixed, downsample to desired NODE_SAMPLING_FREQUENCY
        # Discarding samples to downsample
        if sample_count % (np.round(QC_SAMPLING_FREQUENCY / NODE_SAMPLING_FREQUENCY)) != 0:
            continue

        # TODO: identify noisy channels/freq/etc
        topic_publish_reading(raw_pub, emg_reading)
        data.append(emg_reading)

        # First 128 channels are for IN1..IN8, each MULTIPLE IN has 64 channels
        hdemg_reading = emg_reading[128:128 + MUSCLE_COUNT * 64]
        raw_muscle_reading = []
        for i in range(MUSCLE_COUNT):
            raw_muscle_reading.append(hdemg_reading[i * 64:(i + 1) * 64])

        # TODO: remove noisy channels
        # TODO: notch filter 60hz powerline

        processor.process_reading(raw_muscle_reading)
        processor.publish_reading(processed_pub)

        r.sleep()

    comm.disconnect(q_socket)

    # TODO: save all data
    df = pd.DataFrame(data)
    path = "/home/arch/Downloads"
    filename = 'python_tcp_data.csv'
    df.to_csv(os.path.join(path, filename), index=False)

    import matplotlib.pyplot as plt

    data = np.array(processor.data)
    plt.plot(data[:, 0])
    plt.show()
