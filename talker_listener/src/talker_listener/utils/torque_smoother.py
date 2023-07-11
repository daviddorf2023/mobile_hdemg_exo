import numpy as np

from talker_listener.utils.moving_window import MovingWindow


class TorqueSmoother:
    _window: MovingWindow
    torque_array = []
    offset_torque_array = []

    offset = 0

    def __init__(self):
        self._window = MovingWindow(25)

    def process_reading(self, reading):
        self._window.append(reading)
        smoothed_reading = np.mean(self._window)
        self.torque_array.append(smoothed_reading)
        self.offset_torque_array.append(smoothed_reading - self.offset)

    def reset(self):
        self._window.clear()
        self.torque_array.clear()
        self.offset_torque_array.clear()
        self.offset = 0
