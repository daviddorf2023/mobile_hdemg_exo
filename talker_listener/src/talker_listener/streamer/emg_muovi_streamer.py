import socket
import struct
import numpy as np

NBYTES = 2
REFRESH_RATE = 1 / 32
MUOVI_SAMPLING_FREQUENCY = 512
NUMCYCLES = 20  # Number of data recordings
CONVFACT = 0.000286  # Conversion factor for the bioelectrical signals to get the values in mV
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
        self.t.bind(('0.0.0.0', 54321))
        print('Waiting for connection...')
        self.t.listen(1)
        self.conn, self.addr = self.t.accept()
        print('Connected to the Muovi+ probe')
        self.conn.send(struct.pack('B', 9))  # Send the command to Muovi+ probe

    def close(self):
        self.conn.close()
        self.t.close()
        print("Disconnected from the Muovi+ probe")

    def stream_data(self):
        emg_reading = b''
        emg_reading += self.conn.recv(BLOCKDATA)
        emg_reading = np.frombuffer(emg_reading, dtype=np.int16)  # Read one second of data into signed integer
        emg_reading = emg_reading[:64]
        print(emg_reading)
        return emg_reading
