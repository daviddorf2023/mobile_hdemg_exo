import socket
import rospy

from mobile_hdemg_exo.qc.qc_connect_config import FsampVal, create_connection_confString, \
    create_disconnect_confString

CONVERSION_FACTOR = 0.000286  # Conversion factor needed to get values in mV
SAMPLING_FREQUENCY = rospy.get_param("/sampling_frequency", int)

def read_raw_bytes(connection: socket.socket, number_of_all_channels, bytes_in_sample):
    buffer_size = number_of_all_channels * bytes_in_sample * SAMPLING_FREQUENCY
    new_bytes = connection.recv(buffer_size)
    rospy.set_param("/connected_to_emg", True)
    return new_bytes


# Convert byte-array value to an integer value and apply two's complement
def convert_bytes_to_int(bytes_value):
    value = int.from_bytes(bytes_value, byteorder='little', signed=True)
    return value


# Convert channels from bytes to integers
def bytes_to_integers(
        sample_from_channels_as_bytes,
        number_of_channels,
        bytes_in_sample,
        output_milli_volts):
    channel_values = []
    # Separate channels from byte-string. One channel has
    # "bytes_in_sample" many bytes in it.
    for channel_index in range(number_of_channels):
        channel_start = channel_index * bytes_in_sample
        channel_end = (channel_index + 1) * bytes_in_sample
        channel = sample_from_channels_as_bytes[channel_start:channel_end]

        # Convert channel's byte value to integer
        value = convert_bytes_to_int(channel)  # , bytes_in_sample)
        # MAKE SURE TO CHANGE THIS BACK^^^

        # Convert bio measurement channels to milli volts if needed
        # The 4 last channels (Auxiliary and Accessory-channels)
        # are not to be converted to milli volts
        if output_milli_volts:
            value *= CONVERSION_FACTOR
        channel_values.append(value)
    return channel_values


# ---------- Connection ---------- #

HOST = "169.254.1.10"
TCP_PORT = 23456


# Connect to Quattrocento
def connect(refresh_rate, sampling_frequency, muscle_count) -> socket:
    NumChanSel = muscle_count - 1
    FSampSel = FsampVal.index(sampling_frequency)

    confString = create_connection_confString(FSampSel, NumChanSel)
    # confString[39] = 99 # for MUSCLE_COUNT = 3
    # confString[39] = 241 # for MUSCLE_COUNT = 4
    # confString[39] = 99  # should be equal to the crc8 of ConfString
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, TCP_PORT))
    s.sendall(bytes(confString))

    print(f"Connected to Quattrocento at {HOST}:{TCP_PORT}!")
    print(f"Using refresh_rate={refresh_rate}, "
          f"sampling_frequency={sampling_frequency}, "
          f"muscle_count={muscle_count}!")

    rospy.set_param("connected_to_emg", True)

    return s


# Disconnect from Quattrocento
def disconnect(sock):
    confString = create_disconnect_confString()

    # print(confString)
    sock.sendall(bytes(confString))
    sock.close()

    print(f"Disconnected from Quattrocento at {HOST}:{TCP_PORT}!")
