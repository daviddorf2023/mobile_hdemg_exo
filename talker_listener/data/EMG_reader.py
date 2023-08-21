import csv
import matplotlib.pyplot as plt

x = []
y = []
first_time = None

path = 'pwm_output_50hz.csv'

with open(path, 'r') as file:
    reader = csv.reader(file)
    next(reader)  # skip the first row, contains column names
    for row in reader:
        if first_time is None:
            first_time = int(row[0])
        time = (int(row[0]) - first_time) / 1e9
        corrected_voltage = (float(row[4]) - 1000)/1000
        if 0 < corrected_voltage < 4:
            x.append(time)
            y.append(corrected_voltage)

plt.plot(x, y)
plt.title('EMG signal with 50Hz PWM input')
plt.xlabel('Time [s]')
plt.ylabel('EMG [V]')
plt.show()