#!/usr/bin/env python3

from enum import Enum

import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from talker_listener.msg import hdemg

from talker_listener.processors.emg_process_rms import EMGProcessorRMS

# TODO: Fix scikit-learn issues
# from talker_listener.processors.emg_process_cst import EMGProcessorCST

from talker_listener.streamer.emg_file_streamer import EMGFileStreamer
from talker_listener.streamer.emg_qc_streamer import EMGQCStreamer
from talker_listener.streamer.emg_muovi_streamer import EMGMUOVIStreamer

QC_MUSCLE_COUNT = 3
MUOVI_MUSCLE_COUNT = 1  # TODO: Change based on number of probes
SIM_MUSCLE_COUNT = 4

# Launch file arguments

EMG_DEVICE = rospy.get_param("/device")
EMG_PROCESS_METHOD = rospy.get_param("/method")


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
    if EMG_DEVICE == 'qc':
        streamer = EMGQCStreamer(QC_MUSCLE_COUNT)
        MUSCLE_COUNT = QC_MUSCLE_COUNT
    elif EMG_DEVICE == 'muovi':
        streamer = EMGMUOVIStreamer(MUOVI_MUSCLE_COUNT)
        MUSCLE_COUNT = MUOVI_MUSCLE_COUNT
    elif EMG_DEVICE == 'file':
        path = rospy.get_param("/file_dir")
        path += "/src/talker_listener/raw_emg_34.csv"
        MUSCLE_COUNT = SIM_MUSCLE_COUNT
        streamer = EMGFileStreamer(MUSCLE_COUNT, 512, path)
    streamer.initialize()  # Moved from outside if statement

    processor = None
    if EMG_PROCESS_METHOD == 'rms':
        processor = EMGProcessorRMS()
    elif EMG_PROCESS_METHOD == 'cst':
        processor = EMGProcessorCST()  # TODO: Fix scikit-learn issues

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

        if EMG_DEVICE == 'qc': 
            # First MUSCLE_COUNT * 32 channels are for IN1..IN8, two INs per muscle
            offset = 128
            # Each MULTIPLE IN has 64 channels
            hdemg_reading = emg_reading[offset:offset + MUSCLE_COUNT * 64]
            raw_muscle_reading = []
            for i in range(MUSCLE_COUNT):
                raw_muscle_reading.append(hdemg_reading[i * 64:(i + 1) * 64])
        if EMG_DEVICE == 'muovi': 
            # Each Muovi+ probe has 70 channels
            raw_muscle_reading = []
            # Keep only first 64 channels, last 6 are IMU data
            hdemg_reading = emg_reading[:64]
            for i in range(MUSCLE_COUNT):
                raw_muscle_reading.append(emg_reading)
        
        # TODO: Verify that data processing is accurate by measuring shape of QC packets and compare

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
