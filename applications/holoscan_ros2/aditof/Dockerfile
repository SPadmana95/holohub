# syntax=docker/dockerfile:1

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

############################################################
# Build the aditof image with ROS2 and hololink dependencies
# for ADI ADTF3175 Time-of-Flight sensor ROS 2 publisher/subscriber
############################################################
ARG BASE_IMAGE=nvcr.io/nvidia/clara-holoscan/holoscan:v3.5.0-dgpu
FROM ${BASE_IMAGE} AS aditof

# Force IPv4 for apt — prevents "Network is unreachable" on hosts without IPv6 routing
RUN echo 'Acquire::ForceIPv4 "true";' > /etc/apt/apt.conf.d/99force-ipv4

# Install basic dependencies
RUN apt update && apt install -y \
    curl \
    gnupg \
    lsb-release \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Install git to clone the public holoscan-sensor-bridge repository
RUN apt update && apt install -y \
    git \
    && rm -rf /var/lib/apt/lists/*

# Add ROS2 Jazzy repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy and Vulkan support (required for HolovizOp in subscriber)
# Also install libcurl and libssl required by programmer.cpp (firmware update)
RUN apt update && apt install -y \
    ros-jazzy-desktop \
    ros-jazzy-ros-base \
    vulkan-tools \
    vulkan-validationlayers \
    libvulkan1 \
    libcurl4-openssl-dev \
    libssl-dev \
    libibverbs-dev \
    && rm -rf /var/lib/apt/lists/*

# The apt-installed scipy is compiled against NumPy 1.x but the base image ships
# NumPy 2.x.  CuPy imports cupyx.scipy → scipy.sparse which triggers a
# "numpy.dtype size changed" binary incompatibility at import time.
# --ignore-installed installs a NumPy 2.x-compatible wheel into
# /usr/local/lib/python3.12/dist-packages/ without trying to remove the
# apt-managed package (which has no RECORD file and cannot be uninstalled by pip).
# Python resolves /usr/local/lib before /usr/lib so the new wheel takes precedence.
RUN pip3 install --ignore-installed scipy

# Set ROS2 environment variables
ENV ROS_DISTRO=jazzy
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3
ENV CMAKE_PREFIX_PATH=/opt/ros/jazzy:/opt/nvidia/holoscan
ENV AMENT_PREFIX_PATH=/opt/ros/jazzy
ENV LD_LIBRARY_PATH=/opt/ros/jazzy/lib:$LD_LIBRARY_PATH
ENV PATH=/opt/ros/jazzy/bin:$PATH
ENV PKG_CONFIG_PATH=/opt/ros/jazzy/lib/pkgconfig

# Set Vulkan environment variables for HolovizOp
ENV VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
ENV VK_LAYER_PATH=/usr/share/vulkan/explicit_layer.d
ENV VK_LOADER_LAYERS_ENABLE=1
ENV VK_LOADER_DEBUG=all

# Set PYTHONPATH to include ROS2 packages and holoscan_ros2 Python operators.
# /workspace/holohub is bind-mounted at runtime by the holohub runner, so the
# operators/holoscan_ros2/python path is available even though it is not present
# during the image build.
ENV PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:/opt/nvidia/holoscan/python/lib:/workspace/holohub/operators/holoscan_ros2/python

# Add Holoscan shared libraries to runtime linker path
# Required for libholoscan_core.so, libgxf_*.so etc. at application launch
ENV LD_LIBRARY_PATH=/opt/nvidia/holoscan/lib:$LD_LIBRARY_PATH

# Default X display — points to the primary monitor on the host.
# Passed through to the container when holohub runner mounts /tmp/.X11-unix.
ENV DISPLAY=:0

# Create necessary Vulkan directories and fix ICD path.
#
# The NVIDIA Container Runtime injects:
#   VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
# but on Jetson/Tegra the real ICD lives under tegra/nvidia bind-mount paths.
# Create a symlink so the loader finds the correct driver at runtime.
RUN mkdir -p /usr/share/vulkan/icd.d /usr/share/vulkan/explicit_layer.d && \
    ln -sf /usr/lib/aarch64-linux-gnu/tegra/nvidia_icd.json \
           /usr/share/vulkan/icd.d/nvidia_icd.json

RUN echo -e '#!/bin/bash \n \
set -e \n \
source /opt/ros/jazzy/setup.bash \n \
exec "$@"' > /usr/bin/entrypoint.sh && chmod +x /usr/bin/entrypoint.sh

ENTRYPOINT ["/usr/bin/entrypoint.sh"]

# Clone Holoscan Sensor Bridge at tag 2.5.0 (public upstream)
# (compatible with Holoscan SDK 3.9.0 — main branch requires SDK 4.0+)
RUN mkdir -p /deps
WORKDIR /deps
RUN git clone https://github.com/nvidia-holoscan/holoscan-sensor-bridge.git \
    && cd holoscan-sensor-bridge \
    && git checkout tags/2.5.0

# Patch: add gpio_pin_count=16 for Microchip Polarfire board.
# Without this patch --resetAdcam throws "GPIO is not supported on this Hololink board!"
# The fix exists in the demo_HSB_ToF_ROS2 branch but was not included in the 2.5.0 release tag.
RUN sed -i \
    's|microchip_polarfire_metadata\["board_description"\] = "Microchip Polarfire";|microchip_polarfire_metadata["board_description"] = "Microchip Polarfire";\n    microchip_polarfire_metadata["gpio_pin_count"] = 16;|' \
    /deps/holoscan-sensor-bridge/src/hololink/core/enumerator.cpp

# Build holoscan-sensor-bridge C++ libraries
WORKDIR /deps/holoscan-sensor-bridge
RUN mkdir -p build
WORKDIR /deps/holoscan-sensor-bridge/build
RUN cmake .. && make -j$(nproc)

# Install holoscan-sensor-bridge Python package
WORKDIR /deps/holoscan-sensor-bridge/python
RUN pip3 install -e .

# Set default working directory
WORKDIR /workspace
