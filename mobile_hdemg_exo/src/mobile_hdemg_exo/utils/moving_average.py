class MovingAverage:
    def __init__(self, window_size=100):
        self.window_size = window_size
        self.data = []

    def add_data_point(self, value):
        self.data.append(value)
        if len(self.data) > self.window_size:
            self.data.pop(0)

    def get_smoothed_value(self):
        if not self.data:
            raise ValueError("No data points available for smoothing")

        return sum(self.data) / len(self.data)
