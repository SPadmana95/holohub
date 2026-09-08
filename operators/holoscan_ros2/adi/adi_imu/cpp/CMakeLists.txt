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

# Hololink core (built from source; see the consuming application's Dockerfile)
set(HOLOLINK_ROOT "/deps/holoscan-sensor-bridge")
set(HOLOLINK_BUILD_DIR "${HOLOLINK_ROOT}/build")
set(HOLOLINK_INCLUDE_DIR "${HOLOLINK_ROOT}/src")

link_directories(
    "${HOLOLINK_BUILD_DIR}/src/hololink/core"
    "${HOLOLINK_BUILD_DIR}/src/hololink/common"
)

if(NOT TARGET hololink::hololink)
  add_library(hololink STATIC IMPORTED)
  set_target_properties(hololink PROPERTIES
      IMPORTED_LOCATION "${HOLOLINK_BUILD_DIR}/src/hololink/common/libhololink.a"
  )
  add_library(hololink::hololink ALIAS hololink)
endif()

# SPI driver operator for ADIS16505-2/ADIS16607-2 IMUs over HSB.
add_library(adi_imu STATIC
    adi_imu_op.cpp
)

add_library(holoscan::ops::adi_imu ALIAS adi_imu)

set_property(TARGET adi_imu PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(adi_imu
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${HOLOLINK_INCLUDE_DIR}
)

target_link_libraries(adi_imu
    PUBLIC
        holoscan::core
        hololink::hololink
        hololink_core
        fmt
)

install(TARGETS adi_imu
        COMPONENT holoscan-ops)
