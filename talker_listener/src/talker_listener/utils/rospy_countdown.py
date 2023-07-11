import rospy


class RospyCountdown:
    _start: rospy.Time
    _duration: rospy.Duration

    def __init__(self, duration: rospy.Duration):
        self._duration = duration
        self.reset()

    def reset(self):
        self._start = rospy.Time.now()

    def set_duration(self, duration: rospy.Duration):
        self._duration = duration

    def is_time_left(self) -> bool:
        return rospy.Time.now() < (self._start + self._duration)
