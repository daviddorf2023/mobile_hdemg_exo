import socket
import struct
import numpy as np
import rospy

NBYTES = 2
REFRESH_RATE = 1 / 32
MUOVI_SAMPLING_FREQUENCY = 512
NUMCYCLES = 20  # Number of data recordings
CONVFACT = 0.000286  # Conversion factor for the bioelectrical signals to get the values in mV
NUMCHAN = 70
BLOCKDATA = 2 * NUMCHAN * MUOVI_SAMPLING_FREQUENCY

class EMGMUOVIStreamer:

    def __init__(self, muscle_count: int):
        self._muscle_count = muscle_count
        self._muovi_sockets = []  # List to hold socket connections

        for _ in range(self._muscle_count):
            muovi_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._muovi_sockets.append(muovi_socket)

    @property
    def sample_frequency(self) -> int:
        return MUOVI_SAMPLING_FREQUENCY

    def initialize(self):
        for socket_index, muovi_socket in enumerate(self._muovi_sockets):
            muovi_socket.bind(('0.0.0.0', 54321 + socket_index))
            muovi_socket.listen(1)
            print(f'Waiting for connection on socket {socket_index}...')
            conn, addr = muovi_socket.accept()
            print(f'Connected to the Muovi+ probe on socket {socket_index}')
            conn.send(struct.pack('B', 9))  # Send the command to Muovi+ probe
            rospy.set_param(f"connected_to_emg_{socket_index}", True)

    def close(self):
        for muovi_socket in self._muovi_sockets:
            muovi_socket.close()
        print("Disconnected from the Muovi+ probes")

    def stream_data(self):
        emg_readings_by_socket = {}
        for socket_index, muovi_socket in enumerate(self._muovi_sockets):
            emg_reading = b''
            emg_reading += muovi_socket.recv(BLOCKDATA)
            emg_reading = np.frombuffer(emg_reading, dtype=np.int16)  # Read one second of data into signed integer
            emg_reading = emg_reading * CONVFACT  # Convert to mV
            emg_readings_by_socket[socket_index] = emg_reading
        return emg_readings_by_socket

