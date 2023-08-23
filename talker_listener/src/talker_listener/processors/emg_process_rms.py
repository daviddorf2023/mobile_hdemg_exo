import numpy as np
from talker_listener.utils.moving_window import MovingWindow

class EMGProcessorRMS:
    def process_reading(hdemg_reading):
        """
        Calculates RMS emg.
        """
        sum_squares = sum(x**2 for x in hdemg_reading)
        rms_muscle_reading = (sum_squares / len(hdemg_reading)) ** 0.5
        return rms_muscle_reading
