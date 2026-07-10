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

"""
ADI ADTF3175 ToF Holoscan ROS 2 Publisher (Python)

Replaces HolovizOp from adcam_player.py with three ROS 2 publisher topics.
The sensor is always power-on reset on startup to ensure a clean state.

Pipeline:
  HSB (LinuxReceiverOp / RoceReceiverOp)
    → CsiToBayerOp
    → ADTFUnpackOp  (5-byte/pixel → Depth/AB/Conf rgb8 planes)
    → AdiTofPublisherOp
        ├─ /aditof/depth_image   (sensor_msgs/Image, rgb8 jet-colourised depth)
        ├─ /aditof/ab_image      (sensor_msgs/Image, rgb8 grayscale active brightness)
        └─ /aditof/conf_image    (sensor_msgs/Image, rgb8 grayscale confidence)
"""

import argparse
import ctypes
import logging
import sys

import cupy as cp
import cuda.bindings.driver as cuda
import holoscan
import hololink as hololink_module
import rclpy
from holoscan_ros2.bridge import Bridge
from holoscan_ros2.operator import Operator as ROS2Operator
from sensor_msgs.msg import Image

import adcam
from adcam_unpack_op import ADTFUnpackOp


# ─────────────────────────────────────────────────────────────────────────────
#  AdiTofPublisherOp
#  Receives {"Depth", "ActiveBrightness", "Conf"} rgb8 CuPy tensors from
#  ADTFUnpackOp and publishes each as a separate sensor_msgs/Image topic.
# ─────────────────────────────────────────────────────────────────────────────
class AdiTofPublisherOp(ROS2Operator):
    def __init__(self, fragment, *args, frame_id="aditof_optical", **kwargs):
        self._frame_id = frame_id
        super().__init__(fragment, *args, **kwargs)

    def setup(self, spec):
        spec.input("input")

    def initialize(self):
        super().initialize()
        bridge = self.ros2_bridge()
        self._depth_pub = bridge.create_publisher(Image, "aditof/depth_image", 10)
        self._ab_pub    = bridge.create_publisher(Image, "aditof/ab_image",     10)
        self._conf_pub  = bridge.create_publisher(Image, "aditof/conf_image",   10)

    def _to_image_msg(self, tensor):
        """Convert a CuPy (H, W, 3) uint8 array to sensor_msgs/Image (rgb8)."""
        cpu = cp.asnumpy(tensor)
        h, w = cpu.shape[:2]
        msg = Image()
        msg.header.frame_id = self._frame_id
        msg.height = h
        msg.width  = w
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = w * 3
        msg.data = cpu.tobytes()
        return msg

    def compute(self, op_input, op_output, context):
        tensors = op_input.receive("input")
        self._depth_pub.publish(self._to_image_msg(tensors["Depth"]))
        self._ab_pub.publish(self._to_image_msg(tensors["ActiveBrightness"]))
        self._conf_pub.publish(self._to_image_msg(tensors["Conf"]))


# ─────────────────────────────────────────────────────────────────────────────
#  HoloscanPublisherApp
# ─────────────────────────────────────────────────────────────────────────────
class HoloscanPublisherApp(holoscan.core.Application):
    def __init__(self, cuda_context, cuda_device_ordinal, hololink_channel,
                 ibv_name, ibv_port, adcam_inst, frame_limit):
        super().__init__()
        self._cuda_context = cuda_context
        self._cuda_device_ordinal = cuda_device_ordinal
        self._hololink_channel = hololink_channel
        self._ibv_name = ibv_name
        self._ibv_port = ibv_port
        self._adcam_inst = adcam_inst
        self._frame_limit = frame_limit

    def compose(self):
        # Condition
        if self._frame_limit:
            condition = holoscan.conditions.CountCondition(
                self, name="count", count=self._frame_limit)
        else:
            condition = holoscan.conditions.BooleanCondition(
                self, name="ok", enable_tick=True)

        # CSI → Bayer pool + operator
        csi_pool = holoscan.resources.BlockMemoryPool(
            self, name="csi_pool", storage_type=1,
            block_size=(self._adcam_inst.get_width()
                        * ctypes.sizeof(ctypes.c_uint16)
                        * self._adcam_inst.get_height()),
            num_blocks=2,
        )
        csi_to_bayer = hololink_module.operators.CsiToBayerOp(
            self, name="csi_to_bayer",
            allocator=csi_pool,
            cuda_device_ordinal=self._cuda_device_ordinal,
        )

        # Re-probe + configure inside compose() (mirrors C++ compose() behaviour)
        if not self._adcam_inst.probe_adcam_adtf3175():
            logging.error("ADTF3175 NOT found in compose() — check connections")
            sys.exit(1)
        self._adcam_inst.configure_converter(csi_to_bayer)
        self._adcam_inst.set_mipi()
        self._adcam_inst.set_mode()

        frame_size = csi_to_bayer.get_csi_length()
        logging.info(f"Publisher: frame_size={frame_size}")

        # Receiver (ROCE or Linux)
        if self._ibv_name is not None:
            receiver = hololink_module.operators.RoceReceiverOp(
                self, condition, name="receiver",
                frame_size=frame_size,
                frame_context=self._cuda_context,
                ibv_name=self._ibv_name,
                ibv_port=self._ibv_port,
                hololink_channel=self._hololink_channel,
                device=self._adcam_inst,
            )
        else:
            receiver = hololink_module.operators.LinuxReceiverOperator(
                self, condition, name="receiver",
                frame_size=frame_size,
                frame_context=self._cuda_context,
                hololink_channel=self._hololink_channel,
                device=self._adcam_inst,
            )

        # Unpack 5-byte/pixel → Depth / AB / Conf rgb8 planes
        adtf_unpack = ADTFUnpackOp(
            self, name="ADIToF_data", no_of_planes=3,
            width=self._adcam_inst.get_pixel_width(),
            height=self._adcam_inst.get_pixel_height(),
        )

        # ROS 2 bridge + publisher operator
        bridge = Bridge.from_node_name(
            self, "aditof_publisher_node", name="aditof_publisher_resource")
        ros2_publisher = AdiTofPublisherOp(
            self, bridge, name="aditof_publisher", frame_id="aditof_optical")

        # Pipeline
        self.add_flow(receiver,     csi_to_bayer,  {("output", "input")})
        self.add_flow(csi_to_bayer, adtf_unpack,   {("output", "input")})
        self.add_flow(adtf_unpack,  ros2_publisher, {("output", "input")})


# ─────────────────────────────────────────────────────────────────────────────
#  main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="ADI ADTF3175 Holoscan ROS 2 Publisher"
    )
    parser.add_argument("--captureMode", type=int, default=6,
                        help="ADI capture mode (0-9, default 6)")
    parser.add_argument("--resetPin", type=int, default=0,
                        help="GPIO reset pin (0-31, default 0)")
    parser.add_argument("--hololink", default="192.168.0.2",
                        help="Hololink board IP (default 192.168.0.2)")
    parser.add_argument("--frame-limit", type=int, default=0,
                        help="Stop after N frames (0 = unlimited)")
    parser.add_argument("--log-level", default="info",
                        help="Log level: trace/debug/info/warn/error (default info)")

    infiniband_devices = hololink_module.infiniband_devices()
    if infiniband_devices:
        parser.add_argument("--ibv-name", default=infiniband_devices[0],
                            help="IBV device to use")
        parser.add_argument("--ibv-port", type=int, default=1,
                            help="IBV port number")
    else:
        parser.add_argument("--ibv-name", default=None,
                            help="Network device (None = LinuxReceiverOp)")
        parser.add_argument("--ibv-port", type=int, default=0,
                            help="Port number")

    args = parser.parse_args()

    _log_map = {
        "trace": logging.DEBUG, "debug": logging.DEBUG,
        "info": logging.INFO, "warn": logging.WARNING, "error": logging.ERROR,
    }
    logging.basicConfig(level=_log_map.get(args.log_level.lower(), logging.INFO))

    if not (0 <= args.captureMode <= 9):
        print(f"Error: --captureMode must be 0-9, got {args.captureMode}")
        sys.exit(1)
    if not (0 <= args.resetPin <= 31):
        print(f"Error: --resetPin must be 0-31, got {args.resetPin}")
        sys.exit(1)

    rclpy.init()

    # CUDA init
    (cu_result,) = cuda.cuInit(0)
    assert cu_result == cuda.CUresult.CUDA_SUCCESS, "cuInit failed"
    cu_result, cu_device = cuda.cuDeviceGet(0)
    assert cu_result == cuda.CUresult.CUDA_SUCCESS
    cu_result, cu_context = cuda.cuDevicePrimaryCtxRetain(cu_device)
    assert cu_result == cuda.CUresult.CUDA_SUCCESS

    logging.info("Connecting to Hololink board...")
    channel_metadata = hololink_module.Enumerator.find_channel(channel_ip=args.hololink)
    hololink_channel = hololink_module.DataChannel(channel_metadata)
    adcam_inst = adcam.adcam(
        hololink_channel, hololink_module.CAM_I2C_BUS, channel_metadata,
        adcam_mode=args.captureMode, reset_pin=args.resetPin,
    )

    hololink = hololink_channel.hololink()
    hololink.start()

    # Always power-on reset to ensure a clean sensor state
    logging.info("Performing power-on reset...")
    adcam_inst.adcam_reset_power_on()
    adcam_inst.switch_from_standard_to_burst()
    for label, cmd in [("Master", adcam.GET_MASTER_FIRMWARE_COMMAND),
                       ("Slave",  adcam.GET_SLAVE_FIRMWARE_COMMAND)]:
        v = adcam_inst.get_fw_version_burst_mode(cmd)
        if len(v) >= 4:
            print(f"{label} Firmware version = {v[0]}.{v[1]}.{v[2]}.{v[3]}")
    adcam_inst.switch_from_burst_to_standard()

    if not adcam_inst.probe_adcam_adtf3175():
        logging.warning("ADTF3175 not responding after reset — retrying...")
        adcam_inst.adcam_reset_power_on()
        if not adcam_inst.probe_adcam_adtf3175():
            logging.error("ADTF3175 not found — check connections")
            hololink.stop()
            sys.exit(1)
    logging.info("ADTF3175 Found")

    adcam_inst.get_imager_type_and_ccb_version()

    app = HoloscanPublisherApp(
        cu_context, 0, hololink_channel,
        args.ibv_name, args.ibv_port, adcam_inst, args.frame_limit,
    )
    app.run()

    hololink.stop()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
