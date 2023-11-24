import numpy as np
import rospy
import tensorflow as tf

SAMPLING_FREQUENCY = rospy.get_param("/sampling_frequency", int)


class MUdecomposer(object):
    def __init__(self, model_file=None):
        if model_file == None:
            raise ValueError("No model file specified")
        self.model_file = model_file
        self.model = tf.lite.Interpreter(model_path=model_file)
        self.model.allocate_tensors()
        self.input_details = self.model.get_input_details()
        self.output_details = self.model.get_output_details()

    def predict_MUs(self, hdEMG):
        self.model.set_tensor(self.input_details[0]["index"], hdEMG)
        self.model.invoke()
        result_1 = self.model.get_tensor(self.output_details[0]["index"])
        result_2 = self.model.get_tensor(self.output_details[1]["index"])
        result_3 = self.model.get_tensor(self.output_details[2]["index"])
        result_4 = self.model.get_tensor(self.output_details[3]["index"])
        self.preds = [result_2, result_4, result_3, result_1]
        self.preds_binary = tf.where(np.array(self.preds) >= 0.5, 1., 0.)
        return self.preds_binary


class EMGProcessorCST:
    def __init__(self):
        self.model: MUdecomposer
        model_file = rospy.get_param("/file_dir") + \
            "/src/mobile_hdemg_exo/model/best_model_cnn-allrun5_c8b_mix4-SG0-ST20-WS40-MU[0, 1, 2, 3]_1644222946_f.tflite"
        self.model = MUdecomposer(model_file)
        self.raw_readings = []
        self.cst_readings = []
        self.sample_count = 0
        self.CST_prediction_step_size = 40  # samples

    def process_reading(self, reading):
        self.raw_readings.append(reading)
        self.sample_count += 1
        if self.sample_count % self.CST_prediction_step_size == 0:
            self.calculate_mu()
            return self.cst_readings[-1]

    def calculate_mu(self):
        """
        Predict motor unit activation for each muscle using 'CST_prediction_step_size' samples.
        Implicitly downsamples to 'SAMPLING_FREQUENCY/CST_prediction_step_size'
        """
        # TODO: Sum across all readings and channels for each muscle
        raw_readings = np.array(
            [self.raw_readings[-self.CST_prediction_step_size:]])
        raw_readings = raw_readings.astype(np.float32)
        neural_drive = self.model.predict_MUs(raw_readings)
        neural_drive = neural_drive.numpy()
        cst_muscle_reading = np.sum(neural_drive, axis=(0, 2))
        self.cst_readings.append(cst_muscle_reading)
