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
# Build the a2baudio image with hololink dependencies
# for ADI A2B audio beamforming/waveform visualization
############################################################
ARG BASE_IMAGE=nvcr.io/nvidia/clara-holoscan/holoscan:v4.6.0-cuda13
FROM ${BASE_IMAGE} AS a2baudio

RUN echo 'Acquire::ForceIPv4 "true";' > /etc/apt/apt.conf.d/99force-ipv4

RUN apt update && apt install -y \
    git \
    libcurl4-openssl-dev \
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

# Clone Holoscan Sensor Bridge at tag 2.5.0 (public upstream)
# (compatible with Holoscan SDK 3.9.0 — main branch requires SDK 4.0+)
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
# inside this container. a2baudio is C++-only, so no Python package install.
COPY --chmod=755 holohub /tmp/scripts/
RUN /tmp/scripts/holohub env-info

WORKDIR /workspace
