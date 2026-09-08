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

include(pybind11_add_holohub_module)

# The compiled module is named _adi_imu_op (see PYBIND11_MODULE(_adi_imu_op, ...)
# in adi_imu_op_python.cpp), so the Python module name must match.
pybind11_add_holohub_module(
    CPP_CMAKE_TARGET adi_imu
    PYTHON_MODULE_NAME adi_imu_op
    CLASS_NAME create_imu_operator
    SOURCES adi_imu_op_python.cpp
)
