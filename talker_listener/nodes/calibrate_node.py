#!/usr/bin/env python3

import rospy

from talker_listener.calibration.calibrator import Calibrator
from talker_listener.calibration.trial import Trial, TrialDirection, TrajectoryShape
from talker_listener.calibration.trial_runner import TrialRunner

if __name__ == '__main__':
    while not (rospy.get_param("gui_completed") and rospy.get_param("/connected_to_emg")):
        rospy.sleep(0.1)

    rospy.init_node('calibrate_node')
    print("Calibrating...")

    baseline = Trial(0, TrialDirection.NoDirection, TrajectoryShape.Flat, 0, 25)
    PF0 = Trial(0, TrialDirection.PF, TrajectoryShape.Trapezoid, 0.5, 25)
    PF10 = Trial(0.175, TrialDirection.PF, TrajectoryShape.Trapezoid, 0.5, 25)
    PFn10 = Trial(-0.175, TrialDirection.PF, TrajectoryShape.Trapezoid, 0.5, 25)
    DF0 = Trial(0, TrialDirection.DF, TrajectoryShape.Trapezoid, 0.5, 25)
    DF10 = Trial(0.175, TrialDirection.DF, TrajectoryShape.Trapezoid, 0.5, 25)

    trials = [PF10]

    print("Running Trials...")
    TrialRunner(trials).collect_trial_data()

    print("Getting coefficients...")
    # TODO: Generalize for 3 muscles and multiple trials. See talker_listener/src/talker_listener/calibration/trial_runner.py for the uni-muscle version
    # Calibrator(trials).calibrate()
    # rospy.set_param("calibrated", True)

    print("Calibration complete. EMG coefficients:")
    print(rospy.get_param("emg_coef"))
