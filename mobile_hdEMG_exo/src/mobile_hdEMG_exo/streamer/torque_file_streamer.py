import numpy as np
import pandas as pd


class TorqueFileStreamer:
    _sample_frequency: int

    _file_name: str
    _df: pd.DataFrame
    _data: np.ndarray

    _sample_count: int = 0

    def __init__(self, sample_frequency: int, file_name: str):
        self._sample_frequency = sample_frequency
        self._file_name = file_name

    @property
    def sample_frequency(self) -> int:
        return self._sample_frequency

    def initialize(self):
        df = pd.read_csv(self._file_name)
        self._df = df.iloc[:, 1:].dropna()
        self._data = df.to_numpy()
        print(f"TorqueFileStreamer loaded {self._file_name}, data.shape = {self._data.shape})")

    def close(self):
        pass

    def stream_data(self):
        data = self._data[self._sample_count % len(self._data)]
        self._sample_count += 1
        return data
