# Real Time Torque Predictions to Control the Technaid H3 Ankle Exoskeleton 

## Overview

This branch contains work done over Spring and Summer 2023 using the ROS package for the Technaid H3 Ankle Exoskeleton. The package is designed to run on a laptop and a Raspberry Pi. The laptop runs the talker_listener package, which reads data from the OTB Quattrocento and H3 Ankle Exoskeleton and publishes the data to the ROS network. The Raspberry Pi runs the H3 package, which reads the data from the ROS network and sends torque commands to the exoskeleton. Work is currently being done to improve the torque predictions, reduce latency, and to add a user interface to the package. The main focus is to eventually implement the package on the exoskeleton's onboard computer with the Muovi EMG system, and using ethernet to connect the exoskeleton and EMG system to the computer.

## Latency Analyzer Node
This node uses the Jetson Orin Nano GPIO pin 33 as a PWM output that sends a waveform to the auxilliary input of the EMG processing unit (Quattrocento or Muovi+Pro). It measures the phase difference and divides it by the known frequency of the signal to get the total latency across the entire EMG signal processing stage. The ankle exoskeleton datasheet by Technaid states that the latency of their system is approximately 0.7ms, but this node serves the purpose of measuring the time it takes for the Jetson Orin Nano and EMG system to process signals and generate a torque command from them.

## System Architecture
### 2022 Architecture Block Diagram
![Ankle_H3_Architecture](https://github.com/Technaid-S-L/technaid_h3_ankle_ros_python/assets/113081373/0e073ecc-cda5-4430-9385-55306924cff4)

### 2022 Physical Components
![Router](https://user-images.githubusercontent.com/113081373/235329612-d5902e09-958b-4029-939b-f378cc29b74d.png)

### New System Architecture
![Ankle_H3_Architecture (1)](https://github.com/Technaid-S-L/technaid_h3_ankle_ros_python/assets/113081373/6e45970b-43e6-428e-a39c-b9c8e290a595)

### Latency Debugging Circuit
![Ankle_H3_Architecture](https://github.com/Technaid-S-L/technaid_h3_ankle_ros_python/assets/113081373/ea557a77-9da8-4e65-bdcc-37ac103beec7)
