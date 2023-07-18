import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
import time

TCPPort = 54320
NumCycles = 20  # Number of data recordings
Mode = 0  # 0=64Ch Monop, 1=64Ch with remove average, 2=64Ch ImpCk, 3=64Ch Test
ConvFact = 0.000286  # Conversion factor for the bioelectrical signals to get the values in mV

if Mode > 7:
    print("Error, set ModeX values between 0 and 7")

def dec2bin(n):
    return bin(n).replace("0b", "")

# Create the command to send to Muovi+ probe
Command = 0
Command = Mode * 2 + 9
NumChan = 70
sampFreq = 500
bin(Command).replace("0b", "")  # Convert command to binary

# Open the TCP socket as server
t = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
t.bind(('0.0.0.0', 54321))
print('Waiting for connection...')
t.listen(1)
conn, addr = t.accept()
conn.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 500000)  # Increase the input buffer size
print('Connected to the Muovi+ probe')
conn.send(struct.pack('B', Command))  # Send the command to Muovi+ probe
blockData = 2*NumChan*sampFreq  # Define the length of the data block to be recorded

for i in range(NumCycles):
    # Receive data until the expected size of data is reached
    data = b''
    while len(data) < blockData:
        data += conn.recv(blockData - len(data))
    data = np.frombuffer(data, dtype=np.int16).reshape((NumChan, sampFreq))  # Read one second of data into signed integer

    # Plot the EMG signals
    plt.subplot(2, 1, 1)
    plt.cla()
    plt.title('EMG signals')
    for j in range(NumChan-6):
        plt.plot(data[j, :]*ConvFact + (j-1))
    plt.xlim([0, sampFreq])
    plt.ylim([-1, ((NumChan-6))])

    # Plot the IMU
    plt.subplot(2, 1, 2)
    plt.cla()
    plt.title('IMU signals')
    for j in range(NumChan-5, NumChan-2):
        plt.plot(data[j, :])
    plt.xlim([0, sampFreq])
    plt.ylim([-33000, 33000])

    # Animation
    plt.draw()
    plt.pause(0.001)

# Close the TCP socket
conn.send(struct.pack('B', Command-1))  # Stop the data transfer
time.sleep(0.5)
conn.close()
t.close()
print("Disconnected from the Muovi+ probe")