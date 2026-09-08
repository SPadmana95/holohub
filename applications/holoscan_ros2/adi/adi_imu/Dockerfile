# syntax=docker/dockerfile:1

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
# Build the adi_imu image with ROS2 and hololink dependencies
# for ADI ADIS16505-2/ADIS16607-2 IMU ROS 2 bridge
############################################################
ARG BASE_IMAGE=nvcr.io/nvidia/clara-holoscan/holoscan:v4.6.0-cuda13
FROM ${BASE_IMAGE} AS adi_imu

RUN echo 'Acquire::ForceIPv4 "true";' > /etc/apt/apt.conf.d/99force-ipv4

RUN apt update && apt install -y \
    curl \
    gnupg \
    lsb-release \
    software-properties-common \
    git \
    libcurl4-openssl-dev \
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update && apt install -y \
    ros-jazzy-desktop \
    ros-jazzy-ros-base \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=jazzy
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3
ENV CMAKE_PREFIX_PATH=/opt/ros/jazzy:/opt/nvidia/holoscan
ENV AMENT_PREFIX_PATH=/opt/ros/jazzy
ENV LD_LIBRARY_PATH=/opt/ros/jazzy/lib:/opt/nvidia/holoscan/lib:$LD_LIBRARY_PATH
ENV PATH=/opt/ros/jazzy/bin:$PATH
ENV PKG_CONFIG_PATH=/opt/ros/jazzy/lib/pkgconfig
ENV PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:/opt/nvidia/holoscan/python/lib

RUN echo -e '#!/bin/bash \n \
set -e \n \
source /opt/ros/jazzy/setup.bash \n \
exec "$@"' > /usr/bin/entrypoint.sh && chmod +x /usr/bin/entrypoint.sh

ENTRYPOINT ["/usr/bin/entrypoint.sh"]

# Clone Holoscan Sensor Bridge at tag 2.5.0 (public upstream). The adi_imu
# operator's SPI/GPIO usage is fully supported by this public release, so no
# fork clone is needed; the operator sources live under
# operators/holoscan_ros2/adi_imu/ in this repo.
RUN mkdir -p /deps
WORKDIR /deps
RUN git clone https://github.com/nvidia-holoscan/holoscan-sensor-bridge.git \
    && cd holoscan-sensor-bridge \
    && git checkout tags/2.5.0

# Holoscan's fmt 8 fallback does not expose fmt::char_t as public.
RUN sed -i \
    's|fmt::buffer_context<fmt::char_t<FormatT>>|fmt::format_context|' \
    /deps/holoscan-sensor-bridge/src/hololink/core/logging_internal.hpp

WORKDIR /deps/holoscan-sensor-bridge
RUN mkdir -p build
WORKDIR /deps/holoscan-sensor-bridge/build
RUN cmake .. && make -j$(nproc)

# Install the wrapper-pinned holoscan-cli; the holohub CLI runs `holoscan build`
# inside this container.
COPY --chmod=755 holohub /tmp/scripts/
RUN /tmp/scripts/holohub env-info

WORKDIR /workspace
