import numpy as np
import pandas as pd
import rospy
from scipy import signal

from mobile_hdemg_exo.calibration.trial import Trial
from mobile_hdemg_exo.model.torque_model_v1 import TorqueModelV1


class Calibrator:
    trials: [Trial]

    def __init__(self, trials):
        self.trials = trials

    def calibrate(self):
        emg_df = self.create_df(self.trials)

        nyquist = .5 * 100.0
        b, a = signal.butter(4, .5 / nyquist, btype='lowpass')
        filtered = signal.filtfilt(b, a, emg_df, axis=0).tolist()
        filtered = np.array(filtered)

        emg_df.iloc[:, 2] = filtered[:, 2]
        emg_df.iloc[:, 3] = filtered[:, 3]
        emg_df.iloc[:, 4] = filtered[:, 4]

        path = rospy.get_param("/file_dir")
        emg_df.to_csv(path + "/src/mobile_hdemg_exo/test_data_EMG.csv")

        model = TorqueModelV1()
        emg_coef = model.optimize(emg_df)

        rospy.set_param('emg_coef', emg_coef)

    @staticmethod
    def create_df(trials) -> pd.DataFrame:
        torque_series = np.concatenate([
            t.torque_array - t.baseline_torque
            for t in trials
        ])

        emg_series = np.concatenate([
            signal.resample(t.emg_array, len(t.torque_array))
            for t in trials
        ])
        emg_series = np.swapaxes(emg_series, 0, 1)

        angle_series = np.concatenate([
            np.full(len(t.torque_array), t.joint_angle)
            for t in trials
        ])

        df = pd.concat([
            pd.DataFrame({
                'Torque': torque_series,
                'Angle': angle_series
            }),
            pd.DataFrame({
                f'Muscle {i}': v
                for i, v in enumerate(emg_series)
            })
        ], axis=1)

        return df
