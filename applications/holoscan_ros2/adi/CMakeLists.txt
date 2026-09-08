# SPDX-FileCopyrightText: Copyright (c) 2025-2026, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

# ADI (Analog Devices) sensor applications for holoscan_ros2.
add_holohub_application(aditof DEPENDS OPERATORS holoscan_ros2)
add_holohub_application(a2baudio DEPENDS OPERATORS holoscan_ros2 adi a2b_audio)
add_holohub_application(adi_imu DEPENDS OPERATORS holoscan_ros2 adi adi_imu)
