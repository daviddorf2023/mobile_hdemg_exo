import socket
import struct
import numpy as np
import rospy

MUOVI_SAMPLING_FREQUENCY = 512
CONVFACT = 0.000286  # Conversion factor for the EMG data to get the values in mV
NUMCHAN = 70
BLOCKDATA = 2*NUMCHAN*MUOVI_SAMPLING_FREQUENCY


class EMGMUOVIStreamer:
    """
    Class to stream EMG data from Muovi+ probes

    Attributes
    ----------
    _muovi_socket1 : socket.socket
        Socket to connect to the first Muovi+ probe
    _muovi_socket2 : socket.socket
        Socket to connect to the second Muovi+ probe
    _muscle_count : int
        Number of muscles to stream

    Methods
    -------
    initialize()
        Initialize the connection to the Muovi+ probes
    close()
        Close the connection to the Muovi+ probes
    stream_data()
        Stream the EMG data from the Muovi+ probes
    """

    _muovi_socket1: socket.socket
    # _muovi_socket2: socket.socket

    def __init__(self, muscle_count: int):
        self._muscle_count = muscle_count
        self._muovi_socket1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self._muovi_socket2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    @property
    def sample_frequency(self) -> int:
        """
        Get the sampling frequency of the Muovi+ probes
        """
        return MUOVI_SAMPLING_FREQUENCY

    def initialize(self):
        """
        Initialize the connection to the Muovi+ probes
        """
        self._muovi_socket1.bind(('0.0.0.0', 54321))
        print('Waiting for connection 1...')
        self._muovi_socket1.listen(1)
        self.conn1, self.addr1 = self._muovi_socket1.accept()
        print('Connected to Muovi+ probe 1')
        # Send the command to Muovi+ probe 1
        self.conn1.send(struct.pack('B', 9))

        # self._muovi_socket2.bind(('0.0.0.0', 54321))
        # print('Waiting for connection 2...')
        # self._muovi_socket2.listen(1)
        # self.conn2, self.addr2 = self._muovi_socket2.accept()
        # print('Connected to Muovi+ probe 2')
        # # Send the command to Muovi+ probe 2
        # self.conn2.send(struct.pack('B', 9))

        rospy.set_param("connected_to_emg", True)

    def close(self):
        """
        Close the connection to the Muovi+ probes
        """
        self.conn1.close()
        self._muovi_socket1.close()

        # self.conn2.close()
        # self._muovi_socket2.close()
        print("Disconnected from the Muovi+ probes")

    def stream_data(self):
        """
        Stream the EMG data from the Muovi+ probes
        """
        emg_reading = b''
        emg_reading += self.conn.recv(BLOCKDATA)
        # Read one second of data into signed integer
        emg_reading = np.frombuffer(emg_reading, dtype=np.int16)
        emg_reading = emg_reading * CONVFACT  # Convert to mV
        return emg_reading
