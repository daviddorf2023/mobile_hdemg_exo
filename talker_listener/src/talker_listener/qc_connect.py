"""
Created on Thu Feb  2 13:45:47 2023

@author: jlevine
"""
import socket
from stringprep import c22_specials
import sys
import numpy as np
import time
import csv


# TestDuration = 10;      # Total duration of the test in seconds
# MapsARVEpoch = 0.25;    # Time epoch for the ARV estimation in seconds (must be multiple of RefreshRate)
# RefreshRate = 1 / 32;    # Maps refresh rate in seconds
# ColorScale = 1;         # Set the amplitude in mV corresponding to the white color (max range) in the color plots
# '''
# col = [];
# col[0,:] = [4,5,11,10,24,32,34,39,40,49,50,62,61];            # column no.1
# col[1,:] = [3,6,12,9,23,31,33,38,48,41,51,63,60];             # column no.2
# col[2,:] = [2,7,13,17,22,30,27,37,47,42,52,64,59];            # column no.3
# col[3,:] = [1,8,14,18,21,29,26,36,46,43,53,56,58];            # column no.4
# col[4,:] = [1,16,15,19,20,28,25,35,45,44,54,55,57];           # column no.5'''
# n_column = 5;
# n_row = 12;

# NumRefreshPerEpoch = MapsARVEpoch/RefreshRate;
# NumCycles = TestDuration/RefreshRate;

# Sampling frequency values
Fsamp = [0, 8, 16, 24];
FsampVal = [512, 2048, 5120, 10240];
# FSsel = 1;
'''             
# FSsel  = 0 -> 512 Hz
# FSsel  = 1 -> 2048 Hz
# FSsel  = 2 -> 5120 Hz
# FSsel  = 3 -> 10240 Hz
'''

# Channels numbers
NumChan = [0, 2, 4, 6];
NumChanVal = [120, 216, 312, 408];
# NCHsel = 3;  
'''
# NCHsel = 0 -> IN1, IN2, MULTIPLE IN1, AUX IN
# NCHsel = 1 -> IN1..IN4, MULTIPLE IN1, MULTIPLE IN2, AUX IN
# NCHsel = 2 -> IN1..IN6, MULTIPLE IN1..MULTIPLE IN3, AUX IN
# NCHsel = 3 -> IN1..IN8, MULTIPLE IN1..MULTIPLE IN4, AUX IN
'''

AnOutSource = 0;
'''# Source input for analog output:
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
AnOutChan = 0;
AnOutGain = int('00010000',2);
'''
# int('00000000',2) = Gain on the Analog output equal to 1
# int('00010000',2) = Gain on the Analog output equal to 2
# int('00100000',2) = Gain on the Analog output equal to 4
# int('00110000',2) = Gain on the Analog output equal to 16
'''

GainFactor = 5/2**16/150*1000;
'''
# Provide amplitude in mV
# 5 is the ADC input swing
# 2^16 is the resolution
# 150 is the gain
# 1000 to get the mV
'''

ConfString = [None]*40
# Generate the configuration string
# ConfString[0] = int('10000000',2) + Fsamp[FSsel] + NumChan[NCHsel] + 1;
ConfString[1] = AnOutGain + AnOutSource;
ConfString[2] = AnOutChan;
## -------- IN 1 -------- #
ConfString[3] = 0;
ConfString[4] = 0;
ConfString[5] = int('00010100',2);
## -------- IN 2 -------- #
ConfString[6] = 0;
ConfString[7] = 0;
ConfString[8] = int('00010100',2);
## -------- IN 3 -------- #
ConfString[9] = 0;
ConfString[10] = 0;
ConfString[11] = int('00010100',2);
## -------- IN 4 -------- #
ConfString[12] = 0;
ConfString[13] = 0;
ConfString[14] = int('00010100',2);
# -------- IN 5 -------- #
ConfString[15] = 0;
ConfString[16] = 0;
ConfString[17] = int('00010100',2);
# -------- IN 6 -------- #
ConfString[18] = 0;
ConfString[19] = 0;
ConfString[20] = int('00010100',2);
# -------- IN 7 -------- #
ConfString[21] = 0;
ConfString[22] = 0;
ConfString[23] = int('00010100',2);
# -------- IN 8 -------- #
ConfString[24] = 0;
ConfString[25] = 0;
ConfString[26] = int('00010100',2);
# -------- MULTIPLE IN 1 -------- #
ConfString[27] = 0;
ConfString[28] = 0;
ConfString[29] = int('00010100',2);
# -------- MULTIPLE IN 2 -------- #
ConfString[30] = 0;
ConfString[31] = 0;
ConfString[32] = int('00010100',2);
# -------- MULTIPLE IN 3 -------- #
ConfString[33] = 0;
ConfString[34] = 0;
ConfString[35] = int('00010100',2);
# -------- MULTIPLE IN 4 -------- #
ConfString[36] = 0;
ConfString[37] = 0;
ConfString[38] = int('00010100',2);

# ---------- CRC8 ---------- # Turn into a function
# crc = 0;
# j = 0;
# Len = 39
# Vector = ConfString[0:39]
# inc = np.arange(8,1,-1)

# while(Len > 0):
#     Extract = Vector[j];
#     for i in inc:
        
#         Sum = (crc % 2)^(Extract % 2);
#         crc = int(np.floor(crc/2));
        
#         if(Sum > 0):
#             a = format(crc, '08b');
#             b = format(140, '08b');
#             c = ''
#             for k in range(8):
                
#                 c+=(str(int(a[k] != b[k])))
            
            
#             c = c.replace(" ","")
#             crc = int(c,2);
        
#         Extract = int(np.floor(Extract/2)); 
    
#     Len = Len - 1;
    
#     j=j+1;

# ConfString[39] = 139 #crc


# Accessory channels
# RampChan = NumChanVal[NCHsel]-7;
# BuffChan = NumChanVal[NCHsel]-4;

# Number of samples for each read corresponding to the number of samples of the
# refresh rate
# NumSampBlockRead = FsampVal[FSsel]*RefreshRate;

# number of bytes in sample (2 bytes for the quattrocento device)
nbytes = 2




Host = "192.168.0.10";
TCPPort = 23456;
# Open the TCP socket
# i = 0
# data = []
# tnow = time.time()
# arr = [];

def connect(refresh_rate, FSsel, NCHsel):
    ConfString[0] = int('10000000',2) + Fsamp[FSsel] + NumChan[NCHsel] + 1
    ConfString[39] = 241 # should be equal to the crc8 of ConfString

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((Host,TCPPort))
    s.sendall(bytes(ConfString))

    print(f"Connected to {Host}:{TCPPort}!")
    print(f"Using refresh_rate={refresh_rate}, FSsel={FSsel}, NCHsel{NCHsel}!")

    # while True:
    #     try:
    #         buf = s.recv(2*NumChanVal[NCHsel]*FsampVal[FSsel])
    #         if buf == 'stop':
    #             break
    #         elif len(buf) > 0:
    #             #print(buf)
    #             arr.append(buf)
    #     except:
    #         print('Error')
    #         break

        
    return s, NumChanVal[NCHsel], nbytes


def disconnect(socket):
    ConfString[0] = 128 # stop magic number = 128
    ConfString[39] = 23 # should be equal to the crc8 of ConfString
    s.sendall(bytes(ConfString))
    # socket.close()
