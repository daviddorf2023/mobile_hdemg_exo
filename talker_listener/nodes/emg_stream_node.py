#!/usr/bin/env python3

from enum import Enum

import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from talker_listener.msg import hdemg

# from talker_listener.processors.emg_process_cst import EMGProcessorCST
from talker_listener.processors.emg_process_rms import EMGProcessorRMS
from talker_listener.streamer.emg_file_streamer import EMGFileStreamer
from talker_listener.streamer.emg_qc_streamer import EMGQCStreamer

MUSCLE_COUNT = 4


# EMG streaming methods
class EMGStreamMethod(Enum):
    QC = "qc"
    FILE = "file"


EMG_STREAM_METHOD = EMGStreamMethod.QC


# EMG processing methods
class EMGProcessMethod(Enum):
    RMS = "rms"
    CST = "cst"


EMG_PROCESS_METHOD = EMGProcessMethod.RMS


def topic_publish_reading(publisher: rospy.topics.Publisher, reading: "list[int]"):
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
    rospy.init_node('emg_stream_node')
    raw_pub = rospy.Publisher('hdEMG_stream', hdemg, queue_size=1)
    processed_pub = rospy.Publisher('hdEMG_stream_processed', hdemg, queue_size=1)

    streamer = None
    if EMG_STREAM_METHOD == EMGStreamMethod.QC:
        streamer = EMGQCStreamer(MUSCLE_COUNT)
    elif EMG_STREAM_METHOD == EMGStreamMethod.FILE:
        path = rospy.get_param("/file_dir")
        path += "/src/talker_listener/raw_emg_34.csv"
        streamer = EMGFileStreamer(4, 512, path)
    streamer.initialize()

    processor = None
    if EMG_PROCESS_METHOD == EMGProcessMethod.RMS:
        processor = EMGProcessorRMS()
    elif EMG_PROCESS_METHOD == EMGProcessMethod.CST:
        processor = EMGProcessorCST()

    # data = []
    r = rospy.Rate(streamer.sample_frequency)  # Match the streamer's publishing rate
    while not rospy.is_shutdown():
        emg_reading = streamer.stream_data()
        # # QC_SAMPLING_FREQUENCY is fixed, downsample to desired NODE_SAMPLING_FREQUENCY
        # # Discarding samples to downsample
        # if sample_count % 1 != 0:
        #     continue

        # TODO: identify noisy channels/freq/etc
        topic_publish_reading(raw_pub, emg_reading)
        # data.append(emg_reading)

        # First MUSCLE_COUNT * 32 channels are for IN1..IN8, two INs per muscle
        offset = MUSCLE_COUNT * 32
        # Each MULTIPLE IN has 64 channels
        hdemg_reading = emg_reading[offset:offset + MUSCLE_COUNT * 64]
        raw_muscle_reading = []
        for i in range(MUSCLE_COUNT):
            raw_muscle_reading.append(hdemg_reading[i * 64:(i + 1) * 64])

        # TODO: remove noisy channels
        # TODO: notch filter 60hz powerline

        processor.process_reading(raw_muscle_reading)
        processor.publish_reading(processed_pub)

        r.sleep()

    streamer.close()

    # TODO: save all data
    # df = pd.DataFrame(data)
    # path = "/home/arch/Downloads"
    # filename = 'python_tcp_data.csv'
    # df.to_csv(os.path.join(path, filename), index=False)
    #
    # import matplotlib.pyplot as plt
    #
    # data = np.array(processor.data)
    # plt.plot(data[:, 0])
    # plt.show()
