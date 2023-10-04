import rospy
from qc_predict import MUdecomposer

path = rospy.get_param("/file_dir")
model_file = path + "/src/mobile_hdemg_exo/" + \
    "best_model_cnn-allrun5_c8b_mix4-SG0-ST20-WS40-MU[0, 1, 2, 3]_1644222946_f.lite"
model = MUdecomposer(model_file)
