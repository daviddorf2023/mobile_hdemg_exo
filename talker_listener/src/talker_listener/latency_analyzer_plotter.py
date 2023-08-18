import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file into a pandas DataFrame
df = pd.read_csv("/home/sralexo/exo/src/technaid_h3_ankle_ros_python/talker_listener/src/talker_listener/pwm_output.csv")

# Plot the data
plt.plot(df)

# Show the plot
plt.show()