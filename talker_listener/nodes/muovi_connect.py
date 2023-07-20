import socket
import struct
import numpy as np
import matplotlib.pyplot as plt

NUMCYCLES = 20  # Number of data recordings
CONVFACT = 0.000286  # Conversion factor for the bioelectrical signals to get the values in mV
NUMCHAN = 70
SAMPFREQ = 512

# Open the TCP socket as server
t = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
t.bind(('0.0.0.0', 54321))
print('Waiting for connection...')
t.listen(1)
conn, addr = t.accept()
print('Connected to the Muovi+ probe')
conn.send(struct.pack('B', 9))  # Send the command to Muovi+ probe
blockData = 2*NUMCHAN*SAMPFREQ  # Define the length of the data block to be recorded

# Data processing/plotting
for i in range(NUMCYCLES):
    data = b''
    while len(data) < blockData:
        data += conn.recv(blockData - len(data))  # Receive data until the expected size of data is reached
    data = np.frombuffer(data, dtype=np.int16).reshape((NUMCHAN, SAMPFREQ))  # Read one second of data into signed integer

    # Plot the EMG signals (the first 64 channels)
    plt.subplot(2, 1, 1)
    plt.cla()
    plt.title('EMG signals')
    for j in range(NUMCHAN-6):
        plt.plot(data[j, :]*CONVFACT + (j-1))
    plt.xlim([0, SAMPFREQ])
    plt.ylim([-1, ((NUMCHAN-6))])

    # Plot the IMU (the 3 channels from channel 64 to 67)
    plt.subplot(2, 1, 2)
    plt.cla()
    plt.title('IMU signals')
    for j in range(NUMCHAN-5, NUMCHAN-2):
        plt.plot(data[j, :])
    plt.xlim([0, SAMPFREQ])
    plt.ylim([-33000, 33000])

    # Animation
    plt.draw()
    plt.pause(0.0001)

# Close the TCP socket
conn.close()
t.close()
print("Disconnected from the Muovi+ probe")