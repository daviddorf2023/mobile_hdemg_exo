import numpy as np
from talker_listener.utils.moving_window import MovingWindow

class EMGProcessorRMS:
    window: MovingWindow

    def __init__(self):
        self.window = MovingWindow(10)

    @staticmethod
    def process_reading(window: MovingWindow) -> np.ndarray:
        """
        Calculates RMS emg across time for each channel and average 100 channels together for each muscle.
        """
        smoothed_rms_muscle_reading = np.sqrt(np.mean(np.square(window)))
        avg_muscle_reading = np.mean(smoothed_rms_muscle_reading)
        return avg_muscle_reading
