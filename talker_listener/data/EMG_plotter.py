import csv
import matplotlib.pyplot as plt

x = []
y = []
first_time = None

path = 'new_emg_muovi_data1.csv'

# # QUATTROCENTO
# with open(path, 'r') as file:
#     reader = csv.reader(file)
#     next(reader)  # skip the first row, contains column names
#     for row in reader:
#         if first_time is None:
#             first_time = int(row[0])
#         time = (int(row[0]) - first_time) / 1e9
#         corrected_voltage = (float(row[4]) - 1000)/1000
#         if 0 < corrected_voltage < 4:
#             x.append(time)
#             y.append(corrected_voltage)

# MUOVI+PRO
with open(path, 'r') as file:
    reader = csv.reader(file)
    next(reader)  # skip the first row, contains column names
    for row in reader:
        if first_time is None:
            first_time = int(row[0])
        time = (int(row[0]) - first_time) / 1e9
        x.append(time)
        y.append(float(row[4]))

plt.plot(x, y)
plt.title('EMG signal')
plt.xlabel('Time [s]')
# plt.ylabel('EMG [V]')
plt.ylabel('EMG [Units]') # Unknown conversion factor for Muovi+Pro currently
plt.show()