# Real Time Torque Predictions to Control the Technaid H3 Ankle Exoskeleton 

## Overview

This branch contains work done over Spring and Summer 2023 using the ROS package for the Technaid H3 Ankle Exoskeleton. The package is designed to run on a Jetson Orin Nano, but can also work on any Ubuntu 20.04 system. However, the latency analyzer system only runs on systems with GPIO and PWM pins. The Jetson Orin Nano launches the EMG and ankle exoskeleton ROS packages. The EMG package, ```talker_listener``` connects to and reads data from the OTB Quattrocento, Muovi+ Pro, or simulation dataframes. The H3 packages control and read/write torque data from the exoskeleton. More information about the H3 exoskeleton is in a documentation PDF within this repository. This repository also includes a latency analyzer to measure system processing time across each individual physical component, as well as each program running on the exoskeleton and Orin Nano.

## Dependencies
### Required
- Ubuntu 20.04
- Python 3
- TensorFlow 2.13.0
- SciPy 1.7.0
- NumPy 1.22.4
- Pyttsx3 [Text to speech]
- Tkinter, PyQt [GUI]
### Recommended
- Git
- Pip

## Latency Analyzer Node
This node uses the Jetson Orin Nano GPIO pin 33 as a PWM output that sends a waveform to the auxilliary input of the EMG processing unit (Quattrocento or Muovi+Pro). It measures the phase difference and divides it by the known frequency of the signal to get the total latency across the entire EMG signal processing stage. The ankle exoskeleton datasheet by Technaid states that the latency of their system is approximately 0.7ms, but this node serves the purpose of measuring the time it takes for the Jetson Orin Nano and EMG system to process signals and generate a torque command from them.

## Setup
### Jetson Orin Nano
- Please refer to the setup instructions here: https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit
- The Jetson Orin Nano OS image in the NVIDIA setup is currently a modified version of Ubuntu 20.04 which supports ROS Noetic

### ROS Noetic
- Please refer to the setup instructions here: http://wiki.ros.org/noetic/Installation/Ubuntu

### PCAN
The Technaid H3 Exoskeleton uses PEAK CAN as its CAN interface. The PEAK CAN driver package and the PCAN Basic API must be installed on the device with the following steps:
- Download the Linux driver package from https://www.peak-system.com/Drivers.523.0.html?&L=1
- Download the Linux PCAN Basic API from https://www.peak-system.com/PCAN-Basic.239.0.html?&L=1
- Extract the two downloaded ZIP files and move the two extracted folders to /usr/include/
- In the driver directory, run the following commands in a terminal window to build the driver packages:
  - ```sudo apt-get install libpopt-dev```
  - ```sudo make clean all```
  - ```sudo make install```
  - ```sudo modprobe pcan```

### GPIO Pin Configuration
In order to use PWM and other additional GPIO modifications, the steps below must be taken:
- Download the NVIDIA GPIO interface library with `git clone https://github.com/NVIDIA/jetson-gpio.git`
- In the root of the cloned directory, run `sudo python3 setup.py install`
- Run the following commands to add permissions to access the GPIO user group:
  - `sudo groupadd -f -r gpio`
  - `sudo usermod -a -G gpio your_user_name` and replace your_user_name with the one in your command line interface
  - `sudo cp lib/python/Jetson/GPIO/99-gpio.rules /etc/udev/rules.d/`
  - `sudo udevadm control --reload-rules && sudo udevadm trigger`
  - `sudo /opt/nvidia/jetson-io/jetson-io.py` and change Pin 33 in the pinmux configuration to enable PWM on Pin 33. Save and reboot when prompted for the hardware changes to properly initiate.
  - [Optional] Run `python3 simple_pwm.py` from the Jetson GPIO repository cloned previously to test PWM on Pin 33. If a 3V LED is attached to Pin 33 and GND it should dim and brighten from the changing PWM GPIO output. Alternatively, test the pin with an oscilloscope to read the square wave.

### Enabling Local Ethernet Connection [Quattrocento]
- Go to Network Settings and click the Settings cog next to the wired connection
![image](https://github.com/Technaid-S-L/technaid_h3_ankle_ros_python/assets/129555676/06c2e7d8-c9d6-4c1e-af8d-c4f1bb2bb229)
- Access the IPv4 tab and enable Link-Local Only, then hit Apply
![image](https://github.com/Technaid-S-L/technaid_h3_ankle_ros_python/assets/129555676/8e9aafcb-7f32-4bde-a696-5cb6eadc8b61)


###  Repository
- Create a directory with a subdirectory called ```src```
- Navigate into ```src``` and clone the GitHub repository with ```git clone https://github.com/Technaid-S-L/technaid_h3_ankle_ros_python```
- Switch the Git branch to MSR_Project_2023 with ```git checkout MSR_Project_2023```

### Build
- Go back to the directory containing ```src``` and run ```catkin_make```

### Launch
- Source a ROS workspace with ```source devel/setup.bash```
- Launch the system with ```roslaunch talker_listener h3_launch.launch```. The system can be further configured by the user in the startup screen.

### Previous System Architecture [2022]
#### 2022 Block Diagram
![Ankle_H3_Architecture](https://github.com/Technaid-S-L/technaid_h3_ankle_ros_python/assets/113081373/0e073ecc-cda5-4430-9385-55306924cff4)
#### Physical Components
![Router](https://user-images.githubusercontent.com/113081373/235329612-d5902e09-958b-4029-939b-f378cc29b74d.png)

### New System Architecture [2023]
#### Architecture Block Diagram
![2](https://github.com/Technaid-S-L/technaid_h3_ankle_ros_python/assets/113081373/f5318645-71ee-4055-bbe1-ec8127fc091b)
#### Latency Debugging Circuit
![3](https://github.com/Technaid-S-L/technaid_h3_ankle_ros_python/assets/113081373/55c1f7ca-96dd-4099-8f54-fb197453ab6c)
#### Power Architecture
![exo_power](https://github.com/Technaid-S-L/technaid_h3_ankle_ros_python/assets/113081373/665022e3-4449-47c9-a03d-fe992bb653dc)

