import rospy


class TimescaleAxis:
    _start: float
    axis = []

    def __init__(self):
        self.reset()

    def reset(self):
        self._start = rospy.Time.now().to_sec()
        self.axis.clear()

    def timestamp(self):
        self.axis.append(rospy.Time.now().to_sec() - self._start)
