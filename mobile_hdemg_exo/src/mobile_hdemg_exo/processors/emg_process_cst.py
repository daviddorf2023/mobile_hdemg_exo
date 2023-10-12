import numpy as np
import rospy
from scipy import signal
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from mobile_hdemg_exo.msg import StampedFloat64MultiArray as hdemg

from mobile_hdemg_exo.model.qc_predict import MUdecomposer

CST_prediction_step_size = 40  # samples


# TODO: batch size 20


class EMGProcessorCST:
    raw_readings = []
    sample_count = 0
    norms = []

    cst_readings = []
    model: MUdecomposer

    def __init__(self):
        # Neural Net Set-Up
        path = rospy.get_param("/file_dir")
        model_file = path + "/src/mobile_hdemg_exo/" + \
            "best_model_cnn-allrun5_c8b_mix4-SG0-ST20-WS40-MU[0, 1, 2, 3]_1644222946_f.h5"
        self.model = MUdecomposer(model_file)

    def process_reading(self, reading):
        self.raw_readings.append(reading)
        self.sample_count += 1
        if self.sample_count % CST_prediction_step_size == 0:
            self.norms = self.calculate_norms(self.raw_readings)
            self.calculate_mu()

    @staticmethod
    def calculate_norms(readings):
        # Get max value across all readings and all channels for each muscle
        norms = np.max(readings, axis=(0, 2), initial=0)
        rospy.set_param('emg_norm_vals', [float(x) for x in norms])
        return norms

    def calculate_mu(self):
        """
        Predict motor unit activation for each muscle using 'CST_prediction_step_size' samples.
        Implicitly downsamples to 'SAMPLING_FREQUENCY/CST_prediction_step_size'
        """
        raw_readings = np.array(self.raw_readings[-CST_prediction_step_size:])
        muscle_count = raw_readings.shape[1]

        for i in range(muscle_count):
            raw_readings[:, i, :] /= self.norms[i]

        # Shape input for model (40, 3, 64) -> (3, 40, 64)
        raw_readings = np.swapaxes(raw_readings, 1, 0)

        neural_drive = self.model.predict_MUs(raw_readings)
        neural_drive = neural_drive.numpy()

        # Sum across all readings and channels for each muscle
        cst_muscle_reading = np.sum(neural_drive, axis=(0, 2))
        self.cst_readings.append(cst_muscle_reading)

    # TODO: how often/how many cst_samples?
    def calculate_hanning(self):
        """
        Convolve each muscle's over a 40ms hanning window to estimate the cumulative spike train
        """
        window_hanning = np.hanning(
            np.round(0.2 * 512))  # 512 = NODE_SAMPLING_FREQUENCY

        muscle_count = np.shape(self.cst_readings)[1]
        # Convolve each muscle's activation over the hanning window
        smoothed_cst_readings = np.array([
            np.array(signal.convolve(
                self.cst_readings[:, i], window_hanning, mode='valid'
            ))
            for i in range(muscle_count)
        ])
        smoothed_cst_readings = smoothed_cst_readings.transpose()

        return smoothed_cst_readings

    def publish_reading(self, publisher: rospy.Publisher):
        # reading.shape -> (3, unknown)
        reading = self.calculate_hanning()

        stamped_sample = hdemg()
        stamped_sample.header.stamp = rospy.get_rostime()

        sample = Float64MultiArray()
        sample.data = reading
        sample.layout.dim = [
            MultiArrayDimension("rows", 1, reading.shape[0]),
            MultiArrayDimension("columns", 1, reading.shape[1]),
        ]

        stamped_sample.data = sample
        publisher.publish(stamped_sample)
