# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.20)
project(holoscan_ros2_aditof)

find_package(rclcpp REQUIRED HINTS /opt/ros/jazzy)
find_package(sensor_msgs REQUIRED)

find_package(holoscan 3.0 REQUIRED CONFIG
             PATHS "/opt/nvidia/holoscan" "/workspace/holoscan-sdk/install")

# Enable CUDA language
enable_language(CUDA)
find_package(CUDAToolkit REQUIRED)

# Hololink paths
set(HOLOLINK_ROOT "/deps/holoscan-sensor-bridge")
set(HOLOLINK_BUILD_DIR "${HOLOLINK_ROOT}/build")
set(HOLOLINK_INCLUDE_DIR "${HOLOLINK_ROOT}/src")

# Define hololink library directories
set(HOLOLINK_LIB_DIRS
    "${HOLOLINK_BUILD_DIR}/src/hololink/core"
    "${HOLOLINK_BUILD_DIR}/src/hololink/common"
    "${HOLOLINK_BUILD_DIR}/src/hololink/operators"
    "${HOLOLINK_BUILD_DIR}/src/hololink/operators/csi_to_bayer"
    "${HOLOLINK_BUILD_DIR}/src/hololink/operators/linux_receiver"
    "${HOLOLINK_BUILD_DIR}/src/hololink/operators/roce_receiver"
    "${HOLOLINK_BUILD_DIR}/src/hololink/operators/image_processor"
    "${HOLOLINK_BUILD_DIR}/src/hololink/sensors"
    "${HOLOLINK_BUILD_DIR}/src/hololink/sensors/camera"
)

link_directories(${HOLOLINK_LIB_DIRS})

add_library(hololink STATIC IMPORTED)
set_target_properties(hololink PROPERTIES
    IMPORTED_LOCATION "${HOLOLINK_BUILD_DIR}/src/hololink/common/libhololink.a"
    INTERFACE_LINK_LIBRARIES "CUDA::nvrtc"
)
add_library(hololink::hololink ALIAS hololink)

add_library(csi_to_bayer STATIC IMPORTED)
set_target_properties(csi_to_bayer PROPERTIES
    IMPORTED_LOCATION "${HOLOLINK_BUILD_DIR}/src/hololink/operators/csi_to_bayer/libcsi_to_bayer.a"
    INTERFACE_LINK_LIBRARIES "hololink::hololink"
)
add_library(hololink::csi_to_bayer ALIAS csi_to_bayer)

# ── Publisher executable ──────────────────────────────────────────────────────
# Receives frames from ADI ADTF3175 via HSB (LinuxReceiverOp or RoceReceiverOp)
# and publishes Depth, AB, and Confidence images to ROS 2 topics.
add_executable(holoscan_ros2_adcam_publisher
    adcam_ros2_publisher.cpp
    adcam_unpack_op.cpp
    adcam_unpack_op.cu
    adcam_lib.cpp
    programmer.cpp
    adsd3500_flash.cpp
    compute_crc.cpp
    crc_table.cpp
)

target_link_libraries(holoscan_ros2_adcam_publisher PRIVATE
    holoscan::core
    holoscan::ops::holoscan_ros2
    ${rclcpp_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    CUDA::cudart
    CUDA::cuda_driver
    hololink::hololink
    hololink::csi_to_bayer
    hololink_core
    base_receiver_op
    linux_receiver_op       # LinuxReceiverOp vtable
    linux_receiver          # LinuxReceiver implementation
    roce_receiver
    image_processor
    hololink_sensor         # Adcam sensor base
    hololink_camera_sensor  # Camera sensor
    fmt
    ibverbs                 # InfiniBand verbs (RoCE + IBV device enumeration)
    curl                    # libcurl (used by programmer.cpp for firmware download)
    ssl                     # OpenSSL (EVP_* hash functions in programmer.cpp)
    crypto                  # OpenSSL crypto
)

target_include_directories(holoscan_ros2_adcam_publisher PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${HOLOLINK_INCLUDE_DIR}
    ${HOLOLINK_ROOT}/examples/aditof/cpp
)

target_compile_definitions(holoscan_ros2_adcam_publisher PRIVATE HOLOLINK_AVAILABLE=1)

# ── Subscriber executable ─────────────────────────────────────────────────────
# Subscribes to Depth, AB, and Confidence ROS 2 topics and visualizes them
# side-by-side using HolovizOp.
add_executable(holoscan_ros2_adcam_subscriber
    adcam_ros2_subscriber.cpp
)

target_link_libraries(holoscan_ros2_adcam_subscriber PRIVATE
    holoscan::core
    holoscan::ops::holoscan_ros2
    holoscan::ops::holoviz
    ${rclcpp_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    CUDA::cuda_driver
)

target_include_directories(holoscan_ros2_adcam_subscriber PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS holoscan_ros2_adcam_publisher holoscan_ros2_adcam_subscriber
        DESTINATION ${CMAKE_INSTALL_BINDIR})
