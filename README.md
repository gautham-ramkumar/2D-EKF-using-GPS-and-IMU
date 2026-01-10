# 2D Extended Kalman Filter (EKF) for Vehicle Localization
This repository contains a Python implementation of a 2D Extended Kalman Filter designed to fuse high-frequency IMU data with asynchronous GPS measurements. The project demonstrates how to achieve smooth, centimeter-level vehicle localization by combining the "smoothness" of inertial sensors with the "global accuracy" of GNSS.

# Overview
Standard GPS/GNSS sensors are accurate but operate at low frequencies (e.g., 1–5 Hz) and suffer from signal noise. IMUs operate at high frequencies (e.g., 100+ Hz) but drift over time. This EKF bridges the gap by:
- Predicting the vehicle's state (Position, Heading, Velocity) using IMU data.
- Correcting the state whenever a GPS measurement is available.
- Handling GNSS-denied environments (tunnels) where the filter must rely on pure dead reckoning.

# The Tunnel Effect
The filter is designed to handle signal loss. During dropouts, the error grows quadratically due to IMU drift. Upon exiting a tunnel, the EKF performs a "state correction" to snap back to the global truth, as visualized in the error plots.

# Results
<img width="1332" height="664" alt="image" src="https://github.com/user-attachments/assets/169f8ceb-da05-476b-beac-cda1b55ebcec" />
