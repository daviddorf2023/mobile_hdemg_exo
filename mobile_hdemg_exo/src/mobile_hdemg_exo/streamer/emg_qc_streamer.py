from socket import socket

import mobile_hdemg_exo.qc.qc_communication as comm
from mobile_hdemg_exo.qc.qc_connect_config import NumChanVal

# number of bytes in sample (2 bytes for the quattrocento device)
NBYTES = 2

# refresh rate for quattrocento device (set in OT BioLab Lite)
REFRESH_RATE = 1 / 32

QC_SAMPLING_FREQUENCY = 512


class EMGQCStreamer:
    _muscle_count: int

    _qc_socket: socket
    _sample_count: int = 0

    def __init__(self, muscle_count: int):
        self._muscle_count = muscle_count
        self._qc_socket = comm.connect(
            REFRESH_RATE, QC_SAMPLING_FREQUENCY, self._muscle_count)

    def stream_data(self):
        # with 4 muscles, emg_reading is an array of 408 ints (408 channels = (8*16) + (4*64) + 16 + 8)
        emg_reading = self._process_socket_data(
            self._qc_socket, self._muscle_count)
        self._sample_count += 1
        return emg_reading

    @staticmethod
    def _process_socket_data(q_socket, muscle_count):
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

    @property
    def muscle_count(self) -> int:
        return self._muscle_count

    @property
    def sample_frequency(self) -> int:
        return QC_SAMPLING_FREQUENCY

    def __del__(self):
        comm.disconnect(self._qc_socket)
