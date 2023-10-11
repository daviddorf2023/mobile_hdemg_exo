import socket
import struct
import numpy as np
import rospy

NBYTES = 2
REFRESH_RATE = 1 / 32
MUOVI_SAMPLING_FREQUENCY = 512
NUMCYCLES = 20  # Number of data recordings
# Conversion factor for the bioelectrical signals to get the values in mV
CONVFACT = 0.000286
NUMCHAN = 70
BLOCKDATA = 2*NUMCHAN*MUOVI_SAMPLING_FREQUENCY


class EMGMUOVIStreamer:

    _muovi_socket: socket.socket

    def __init__(self, muscle_count: int):
        self._muscle_count = muscle_count
        self.t = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    @property
    def sample_frequency(self) -> int:
        return MUOVI_SAMPLING_FREQUENCY


def initialize(self):
    for socket_index, muovi_socket in enumerate(self._muovi_sockets):
        if socket_index == 0:
            muovi_socket.bind(('0.0.0.0', 54321))
        else:
            # Replace with the correct IP address
            muovi_socket.bind(('192.168.1.100', 54321 + socket_index))
        muovi_socket.listen(1)
        print(f'Waiting for connection on socket {socket_index}...')
        conn, addr = muovi_socket.accept()
        print(f'Connected to the Muovi+ probe on socket {socket_index}')
        conn.send(struct.pack('B', 9))  # Send the command to Muovi+ probe
        rospy.set_param(f"connected_to_emg_{socket_index}", True)

    def close(self):
        self.conn.close()
        self.t.close()
        print("Disconnected from the Muovi+ probe")

    def stream_data(self):
        emg_reading = b''
        emg_reading += self.conn.recv(BLOCKDATA)
        # Read one second of data into signed integer
        emg_reading = np.frombuffer(emg_reading, dtype=np.int16)
        emg_reading = emg_reading * CONVFACT  # Convert to mV
        return emg_reading
