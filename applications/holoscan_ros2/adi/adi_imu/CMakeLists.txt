# adi_imu: ROS2 publisher for ADI ADIS16505-2/ADIS16607-2 IMU sensors.
# The native operator and its Python bindings are built into the container
# image (see Dockerfile); this application is a Python-only ROS 2 script.
cmake_minimum_required(VERSION 3.20)
project(adi_imu LANGUAGES NONE)
