import socket
import struct
import numpy as np

NBYTES = 2
REFRESH_RATE = 1 / 32
MUOVI_SAMPLING_FREQUENCY = 512
NUMCYCLES = 20  # Number of data recordings
CONVFACT = 0.000286  # Conversion factor for the bioelectrical signals to get the values in mV
NUMCHAN = 70

class EMGMUOVIStreamer:

    _muovi_socket: socket.socket
    _sample_count: int = 0

    def __init__(self, muscle_count: int):
        self._muscle_count = muscle_count
        self.t = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.conn, self.addr = self.t.accept()
        self.blockData = 2*NUMCHAN*MUOVI_SAMPLING_FREQUENCY  # Define the length of the data block to be recorded

    @property
    def sample_frequency(self) -> int:
        return MUOVI_SAMPLING_FREQUENCY

    def connect(self):
        self.t.bind(('0.0.0.0', 54321))
        print('Waiting for connection...')
        self.t.listen(1)
        print('Connected to the Muovi+ probe')
        self.conn.send(struct.pack('B', 9))  # Send the command to Muovi+ probe

    def disconnect(self):
        self.conn.close()
        self.t.close()
        print("Disconnected from the Muovi+ probe")

    def stream_data(self, blockData):
        emg_reading = b''
        while len(emg_reading) < blockData:
            emg_reading += self.conn.recv(blockData - len(emg_reading))  # Receive data until the expected size of data is reached
        emg_reading = np.frombuffer(emg_reading, dtype=np.int16).reshape((NUMCHAN, MUOVI_SAMPLING_FREQUENCY))  # Read one second of data into signed integer
        self._sample_count += 1
        return emg_reading
