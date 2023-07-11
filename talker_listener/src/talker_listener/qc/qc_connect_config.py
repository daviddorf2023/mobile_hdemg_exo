"""
Created on Thu Feb  2 13:45:47 2023

@author: jlevine
"""

from talker_listener.utils.crc8 import crc8

# TestDuration = 10      # Total duration of the test in seconds
# MapsARVEpoch = 0.25    # Time epoch for the ARV estimation in seconds (must be multiple of RefreshRate)
# RefreshRate = 1 / 32    # Maps refresh rate in seconds
# ColorScale = 1         # Set the amplitude in mV corresponding to the white color (max range) in the color plots
# '''
# col = []
# col[0,:] = [4,5,11,10,24,32,34,39,40,49,50,62,61]            # column no.1
# col[1,:] = [3,6,12,9,23,31,33,38,48,41,51,63,60]             # column no.2
# col[2,:] = [2,7,13,17,22,30,27,37,47,42,52,64,59]            # column no.3
# col[3,:] = [1,8,14,18,21,29,26,36,46,43,53,56,58]            # column no.4
# col[4,:] = [1,16,15,19,20,28,25,35,45,44,54,55,57]           # column no.5'''
# n_column = 5----------
# n_row = 12

# NumRefreshPerEpoch = MapsARVEpoch/RefreshRate
# NumCycles = TestDuration/RefreshRate

# Sampling frequency values (set in OT BioLab Light)
Fsamp = [0, 8, 16, 24]
FsampVal = [512, 2048, 5120, 10240]
# FSampSel = 1
'''             
# FSampSel  = 0 -> 512 Hz
# FSampSel  = 1 -> 2048 Hz
# FSampSel  = 2 -> 5120 Hz
# FSampSel  = 3 -> 10240 Hz
'''

# Number of Channels
# IN1..IN8 has 16 channels each
# MULTIPLE IN1..MULTIPLE IN4 has 64 channels each
# 16 + 8 channels for AUX I
# For 4 muscles we use MULTIPLE IN1..MULTIPLE IN4 (NumChanSel = 3)
# (4 * 64) + (8 * 16) + (16 + 8) = 408 for the quattrocento device
NumChan = [0, 2, 4, 6]
NumChanVal = [120, 216, 312, 408]
'''
# NumChanSel = 0 -> IN1, IN2, MULTIPLE IN1, AUX IN
# NumChanSel = 1 -> IN1..IN4, MULTIPLE IN1, MULTIPLE IN2, AUX IN
# NumChanSel = 2 -> IN1..IN6, MULTIPLE IN1..MULTIPLE IN3, AUX IN
# NumChanSel = 3 -> IN1..IN8, MULTIPLE IN1..MULTIPLE IN4, AUX IN
'''

AnOutSource = 0
'''
Source input for analog output:
# 0 = the analog output signal came from IN1
# 1 = the analog output signal came from IN2
# 2 = the analog output signal came from IN3
# 3 = the analog output signal came from IN4
# 4 = the analog output signal came from IN5
# 5 = the analog output signal came from IN6
# 6 = the analog output signal came from IN7
# 7 = the analog output signal came from IN8
# 8 = the analog output signal came from MULTIPLE IN1
# 9 = the analog output signal came from MULTIPLE IN2
# 10 = the analog output signal came from MULTIPLE IN3
# 11 = the analog output signal came from MULTIPLE IN4
# 12 = the analog output signal came from AUX IN
'''
# Channel for analog output
AnOutChan = 0
AnOutGain = int('00010000', 2)
'''
# int('00000000',2) = Gain on the Analog output equal to 1
# int('00010000',2) = Gain on the Analog output equal to 2
# int('00100000',2) = Gain on the Analog output equal to 4
# int('00110000',2) = Gain on the Analog output equal to 16
'''

GainFactor = 5 / 2 ** 16 / 150 * 1000
'''
# Provide amplitude in mV
# 5 is the ADC input swing
# 2^16 is the resolution
# 150 is the gain
# 1000 to get the mV
'''


def create_confString(value):
    ConfString = [0] * 40
    # Generate the configuration string
    ConfString[0] = value
    ConfString[1] = AnOutGain + AnOutSource
    ConfString[2] = AnOutChan
    # -------- IN 1 -------- #
    ConfString[3] = 0
    ConfString[4] = 0
    ConfString[5] = int('00010100', 2)
    # -------- IN 2 -------- #
    ConfString[6] = 0
    ConfString[7] = 0
    ConfString[8] = int('00010100', 2)
    # -------- IN 3 -------- #
    ConfString[9] = 0
    ConfString[10] = 0
    ConfString[11] = int('00010100', 2)
    # -------- IN 4 -------- #
    ConfString[12] = 0
    ConfString[13] = 0
    ConfString[14] = int('00010100', 2)
    # -------- IN 5 -------- #
    ConfString[15] = 0
    ConfString[16] = 0
    ConfString[17] = int('00010100', 2)
    # -------- IN 6 -------- #
    ConfString[18] = 0
    ConfString[19] = 0
    ConfString[20] = int('00010100', 2)
    # -------- IN 7 -------- #
    ConfString[21] = 0
    ConfString[22] = 0
    ConfString[23] = int('00010100', 2)
    # -------- IN 8 -------- #
    ConfString[24] = 0
    ConfString[25] = 0
    ConfString[26] = int('00010100', 2)
    # -------- MULTIPLE IN 1 -------- #
    ConfString[27] = 0
    ConfString[28] = 0
    ConfString[29] = int('00010100', 2)
    # -------- MULTIPLE IN 2 -------- #
    ConfString[30] = 0
    ConfString[31] = 0
    ConfString[32] = int('00010100', 2)
    # -------- MULTIPLE IN 3 -------- #
    ConfString[33] = 0
    ConfString[34] = 0
    ConfString[35] = int('00010100', 2)
    # -------- MULTIPLE IN 4 -------- #
    ConfString[36] = 0
    ConfString[37] = 0
    ConfString[38] = int('00010100', 2)

    # ---------- CRC8 ----------
    ConfString[39] = crc8(ConfString, 39)
    return ConfString


def create_connection_confString(FSampSel, NumChanSel):
    return create_confString(int('10000000', 2) + Fsamp[FSampSel] + NumChan[NumChanSel] + 1)


def create_disconnect_confString():
    return create_confString(128)
