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

find_package(holoscan 3.0 REQUIRED CONFIG
             PATHS "/opt/nvidia/holoscan" "/workspace/holoscan-sdk/install")

enable_language(CUDA)
find_package(CUDAToolkit REQUIRED)

# Hololink core (built from source; see the consuming application's Dockerfile)
set(HOLOLINK_ROOT "/deps/holoscan-sensor-bridge")
set(HOLOLINK_BUILD_DIR "${HOLOLINK_ROOT}/build")
set(HOLOLINK_INCLUDE_DIR "${HOLOLINK_ROOT}/src")

link_directories(
    "${HOLOLINK_BUILD_DIR}/src/hololink/core"
    "${HOLOLINK_BUILD_DIR}/src/hololink/common"
    "${HOLOLINK_BUILD_DIR}/src/hololink/operators"
    "${HOLOLINK_BUILD_DIR}/src/hololink/operators/linux_receiver"
)

if(NOT TARGET hololink::hololink)
  add_library(hololink STATIC IMPORTED)
  set_target_properties(hololink PROPERTIES
      IMPORTED_LOCATION "${HOLOLINK_BUILD_DIR}/src/hololink/common/libhololink.a"
      INTERFACE_LINK_LIBRARIES "CUDA::nvrtc"
  )
  add_library(hololink::hololink ALIAS hololink)
endif()

# Waveform, beamformer, and file-writer operators for HSB I2S/A2B audio capture.
# i2s_receiver_op.cpp/.hpp are excluded: they use a Hololink::I2s API that only
# exists in the ADI fork, not the public holoscan-sensor-bridge release.
add_library(a2b_audio STATIC
    audio_waveform_op.cu
    audio_beamformer_op.cpp
    audio_beamformer_op.cu
    audio_file_writer_op.cpp
)

add_library(holoscan::ops::a2b_audio ALIAS a2b_audio)

set_target_properties(a2b_audio PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CUDA_SEPARABLE_COMPILATION ON
)

target_include_directories(a2b_audio
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${HOLOLINK_INCLUDE_DIR}
)

target_link_libraries(a2b_audio
    PUBLIC
        holoscan::core
        CUDA::cudart
        cufft
        hololink::hololink
        hololink_core
        linux_receiver_op
        linux_receiver
        base_receiver_op
        fmt
)
