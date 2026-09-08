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

cmake_minimum_required(VERSION 3.20)
project(holoscan_ros2_a2baudio)

find_package(holoscan 3.0 REQUIRED CONFIG
             PATHS "/opt/nvidia/holoscan" "/workspace/holoscan-sdk/install")

enable_language(CUDA)
find_package(CUDAToolkit REQUIRED)

# Hololink paths (built from source; see Dockerfile)
set(HOLOLINK_ROOT "/deps/holoscan-sensor-bridge")
set(HOLOLINK_BUILD_DIR "${HOLOLINK_ROOT}/build")
set(HOLOLINK_INCLUDE_DIR "${HOLOLINK_ROOT}/src")

set(HOLOLINK_LIB_DIRS
    "${HOLOLINK_BUILD_DIR}/src/hololink/core"
    "${HOLOLINK_BUILD_DIR}/src/hololink/common"
    "${HOLOLINK_BUILD_DIR}/src/hololink/operators"
    "${HOLOLINK_BUILD_DIR}/src/hololink/operators/linux_receiver"
    "${HOLOLINK_BUILD_DIR}/examples/adi/a2baudio/audio_waveform"
    "${HOLOLINK_BUILD_DIR}/examples/adi/a2baudio/audio_beamformer"
    "${HOLOLINK_BUILD_DIR}/examples/adi/a2baudio/audio_filewriter"
)

link_directories(${HOLOLINK_LIB_DIRS})

add_library(hololink STATIC IMPORTED)
set_target_properties(hololink PROPERTIES
    IMPORTED_LOCATION "${HOLOLINK_BUILD_DIR}/src/hololink/common/libhololink.a"
    INTERFACE_LINK_LIBRARIES "CUDA::nvrtc"
)
add_library(hololink::hololink ALIAS hololink)

# ── audio_viz_disp executable ─────────────────────────────────────────────────
# Captures I2S audio via HSB, beamforms it, and visualizes waveforms with HolovizOp.
add_executable(audio_viz_disp
    audio_viz.cpp
    audio_waveform_op.cu
    audio_beamformer_op.cpp
    audio_beamformer_op.cu
    audio_file_writer_op.cpp
)

target_link_libraries(audio_viz_disp PRIVATE
    holoscan::core
    holoscan::ops::holoviz
    CUDA::cudart
    CUDA::cuda_driver
    cufft
    hololink::hololink
    hololink_core
    linux_receiver_op
    linux_receiver
    base_receiver_op
    fmt
)

target_include_directories(audio_viz_disp PRIVATE
    ${HOLOLINK_INCLUDE_DIR}
)

set_target_properties(audio_viz_disp PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CUDA_SEPARABLE_COMPILATION ON
)

install(TARGETS audio_viz_disp DESTINATION ${CMAKE_INSTALL_BINDIR})
