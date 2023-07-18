import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
import time


TCPPort = 54320
NumCycles = 10
OffsetEMG = 1
PlotTime = 1

# Refer to the communication protocol for details about these variables:
ProbeEN = 1  # 1=Probe enabled, 0 = probe disabled
EMG = 0  # 1=EMG, 0=EEG
Mode = 0  # 0=64Ch Monop, 1=64Ch with remove average, 2=64Ch ImpCk, 3=64Ch Test

# Conversion factor for the bioelectrical signals to get the values in mV
ConvFact = 0.000286

# Number of acquired channel depending on the acquisition mode
NumChanVsMode = [70, 70, 70, 70]

if ProbeEN > 1:
    print("Error, set ProbeEN values equal to 0 or 1")
    exit()

if EMG > 1:
    print("Error, set EMGX values equal to 0 or 1")
    exit()

if Mode > 7:
    print("Error, set ModeX values between 0 and 7")
    exit()

# Create the command to send to Muovi+Pro
Command = 0
if ProbeEN == 1:
    Command = 0 + EMG * 8 + Mode * 2 + 1
    NumChan = NumChanVsMode[Mode]
    if EMG == 0:
        sampFreq = 500
    else:
        sampFreq = 2000

# Open the TCP socket as server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('0.0.0.0', 54321))
server_socket.listen(1)

print('Waiting for a connection...')
client_socket, address = server_socket.accept()
print('Connected to the Socket')

plt.subplot(3, 1, 1)
EMGPlot, = plt.plot(0)
plt.xlim([0, sampFreq])
plt.ylim([-OffsetEMG, (OffsetEMG * (NumChan - 6))])

plt.subplot(3, 1, 2)
IMU, = plt.plot(0)
plt.xlim([0, sampFreq])
plt.ylim([-33000, 33000])

plt.subplot(3, 3, 7)
Buf, = plt.plot(0)
plt.xlim([0, sampFreq])
plt.ylim([0, 16000])

plt.subplot(3, 3, 8)
Trig, = plt.plot(0)
plt.xlim([0, sampFreq])
plt.ylim([0, 1.1])

plt.subplot(3, 3, 9)
Ramp, = plt.plot(0)
plt.xlim([0, sampFreq])
plt.ylim([-33000, 33000])

# Send the command to Muovi+Pro
client_socket.send(struct.pack('B', Command))

if ProbeEN == 0:
    # Wait to be sure the command is received before closing the socket
    time.sleep(0.5)
    client_socket.close()
    server_socket.close()
    exit()

# If the high resolution mode (24 bits) is active
if EMG == 0:
    # one second of data: 3 bytes * channels * Sampling frequency
    blockData = 3 * NumChan * sampFreq
    data = b''  # Initialize data variable

    ChInd = np.arange(0, NumChan * 3, 3)

    # Main plot loop
    for i in range(NumCycles):
        print(i)

        # Wait here until one second of data has been received
        while len(data) < blockData:
            data += client_socket.recv(500000)

        # Read one second of data into single bytes
        Temp = np.frombuffer(data, dtype=np.uint8).reshape((NumChan * 3, -1))

        # Combine 3 bytes to a 24 bit value
        data_i = Temp[ChInd, :] * 65536 + Temp[ChInd + 1, :] * 256 + Temp[ChInd + 2, :]

        # Search for the negative values and make the two's complement
        ind = np.where(data_i >= 8388608)
        data_i[ind] = data_i[ind] - 16777216

        # Plot the EMG signals
        plt.subplot(3, 1, 1)
        plt.cla()
        for j in range(NumChan - 6):
            plt.plot(data_i[j, :] * ConvFact + OffsetEMG * (j - 1))

        # Plot the IMU
        plt.subplot(3, 1, 2)
        plt.cla()
        for j in range(NumChan - 5, NumChan - 2):
            plt.plot(data_i[j, :])

        Trigger = np.zeros(sampFreq)
        ind = np.where(data_i[NumChan - 1, :] < 0)
        Trigger[ind] = 1
        Buffer = data_i[NumChan - 1, :]
        print(Buffer)
        Buffer[ind] = data_i[NumChan - 1, ind] + 32768

        plt.subplot(3, 3, 7)
        Buf.set_ydata(Buffer)

        plt.subplot(3, 3, 8)
        Trig.set_ydata(Trigger)

        plt.subplot(3, 3, 9)
        # Ramp.set_ydata(data_i[NumChan, :])

        plt.draw()
        plt.pause(0.01)

    # Stop the data transfer and eventually the recording on MicroSD card
    client_socket.send(struct.pack('B', Command - 1))

else:
    # If the low resolution mode (16 bits) is active

    # one second of data: 2 bytes * channels * Sampling frequency
    blockData = 2 * NumChan * sampFreq
    data = b''  # Initialize data variable

    # Main plot loop
    for i in range(NumCycles):
        print(i)
        print("16 bit resolution mode")

        # Wait here until one second of data has been received
        while len(data) < blockData:
            data += client_socket.recv(500000)

        # Read one second of data into signed integer
        data_i = np.frombuffer(data, dtype=np.int16).reshape((NumChan, -1))

        # Plot the EMG signals
        plt.subplot(3, 1, 1)
        plt.cla()
        for j in range(NumChan - 6):
            plt.plot(data_i[j, :] * ConvFact + OffsetEMG * (j - 1))

        # Plot the IMU
        plt.subplot(3, 1, 2)
        plt.cla()
        for j in range(NumChan - 5, NumChan - 2):
            plt.plot(data_i[j, :])

        Trigger = np.zeros(sampFreq)
        ind = np.where(data_i[NumChan - 1, :] < 0)
        Trigger[ind] = 1

        Buffer = np.array(data_i[NumChan - 1, :])
        print(Buffer)
        Buffer[ind] = data_i[NumChan - 1, ind] + 32768

        plt.subplot(3, 3, 7)
        Buf.set_ydata(Buffer)

        plt.subplot(3, 3, 8)
        Trig.set_ydata(Trigger)

        plt.subplot(3, 3, 9)
        # Ramp.set_ydata(data_i[NumChan, :])

        plt.draw()
        plt.pause(0.001)

    # Stop the data transfer and eventually the recording on MicroSD card
    client_socket.send(struct.pack('B', Command - 1))

# Wait to be sure the command is received before closing the socket
time.sleep(0.5)

# Close the TCP socket
client_socket.close()
server_socket.close()
