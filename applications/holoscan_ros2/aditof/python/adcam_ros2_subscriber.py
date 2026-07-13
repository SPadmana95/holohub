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
ADI ADTF3175 ToF Holoscan ROS 2 Subscriber (Python)

Subscribes to three ROS 2 topics published by adcam_ros2_publisher.py
and displays them side-by-side using HolovizOp.

  Left   (0-33%)  /aditof/depth_image        — jet-colourised depth
  Centre (33-66%) /aditof/ab_image            — grayscale active brightness
  Right  (66-100%)/aditof/conf_image          — grayscale confidence
"""

import argparse
import concurrent.futures
import logging

import cupy as cp
import holoscan
import rclpy
from holoscan_ros2.bridge import Bridge
from holoscan_ros2.operators.subscriber import SubscriberOp
from sensor_msgs.msg import Image


# ─────────────────────────────────────────────────────────────────────────────
#  AdiTofSubscriberOp
#  Subscribes to one sensor_msgs/Image topic, transfers the payload to GPU
#  as a CuPy (H, W, 3) uint8 tensor, and emits it for HolovizOp.
# ─────────────────────────────────────────────────────────────────────────────
class AdiTofSubscriberOp(SubscriberOp):
    def __init__(self, fragment, *args, tensor_name, **kwargs):
        self._tensor_name = tensor_name
        super().__init__(fragment, *args, message_type=Image, **kwargs)

    def setup(self, spec):
        spec.output("output")

    def compute(self, op_input, op_output, context):
        # Wait for a message; retry on timeout so we handle a publisher that
        # starts after the subscriber, or one that temporarily stops.
        while rclpy.ok():
            future = self.receive()
            try:
                msg = future.result(timeout=0.5)
                break
            except concurrent.futures.TimeoutError:
                logging.debug(
                    "AdiTofSubscriberOp [%s]: waiting for publisher...",
                    self._tensor_name,
                )
                continue

        if not rclpy.ok():
            logging.info(
                "AdiTofSubscriberOp [%s]: ROS 2 shutdown detected.",
                self._tensor_name,
            )
            return

        # Convert sensor_msgs/Image (rgb8, H×W×3 uint8) to CuPy device tensor
        h, w = msg.height, msg.width
        arr = cp.frombuffer(bytes(msg.data), dtype=cp.uint8).reshape(h, w, 3)

        op_output.emit({self._tensor_name: arr}, "output")


# ─────────────────────────────────────────────────────────────────────────────
#  HoloscanSubscriberApp
# ─────────────────────────────────────────────────────────────────────────────
class HoloscanSubscriberApp(holoscan.core.Application):
    def __init__(self, headless=False, fullscreen=False):
        super().__init__()
        self._headless = headless
        self._fullscreen = fullscreen

    def compose(self):
        bridge = Bridge.from_node_name(
            self, "aditof_subscriber_node", name="aditof_subscriber_resource"
        )

        depth_sub = AdiTofSubscriberOp(
            self,
            bridge,
            name="depth_subscriber",
            tensor_name="Depth",
            topic_name="aditof/depth_image",
            qos=10,
            message_queue_max_size=1,
        )
        ab_sub = AdiTofSubscriberOp(
            self,
            bridge,
            name="ab_subscriber",
            tensor_name="ActiveBrightness",
            topic_name="aditof/ab_image",
            qos=10,
            message_queue_max_size=1,
        )
        conf_sub = AdiTofSubscriberOp(
            self,
            bridge,
            name="conf_subscriber",
            tensor_name="Conf",
            topic_name="aditof/conf_image",
            qos=10,
            message_queue_max_size=1,
        )

        # HolovizOp — side-by-side layout (mirrors adcam_player.py)
        depth_spec = holoscan.operators.HolovizOp.InputSpec(
            "Depth", holoscan.operators.HolovizOp.InputType.COLOR
        )
        v = holoscan.operators.HolovizOp.InputSpec.View()
        v.offset_x, v.offset_y, v.width, v.height = 0.0, 0.0, 0.33, 1.0
        depth_spec.views = [v]

        ab_spec = holoscan.operators.HolovizOp.InputSpec(
            "ActiveBrightness", holoscan.operators.HolovizOp.InputType.COLOR
        )
        v2 = holoscan.operators.HolovizOp.InputSpec.View()
        v2.offset_x, v2.offset_y, v2.width, v2.height = 0.33, 0.0, 0.33, 1.0
        ab_spec.views = [v2]

        conf_spec = holoscan.operators.HolovizOp.InputSpec(
            "Conf", holoscan.operators.HolovizOp.InputType.COLOR
        )
        v3 = holoscan.operators.HolovizOp.InputSpec.View()
        v3.offset_x, v3.offset_y, v3.width, v3.height = 0.66, 0.0, 0.34, 1.0
        conf_spec.views = [v3]

        visualizer = holoscan.operators.HolovizOp(
            self,
            name="holoviz",
            headless=self._headless,
            fullscreen=self._fullscreen,
            framebuffer_srgb=True,
            tensors=[depth_spec, ab_spec, conf_spec],
            window_title="ADI ToF Subscriber",
        )

        self.add_flow(depth_sub, visualizer, {("output", "receivers")})
        self.add_flow(ab_sub, visualizer, {("output", "receivers")})
        self.add_flow(conf_sub, visualizer, {("output", "receivers")})


# ─────────────────────────────────────────────────────────────────────────────
#  main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="ADI ADTF3175 Holoscan ROS 2 Subscriber")
    parser.add_argument("--headless", action="store_true", help="Run without display window")
    parser.add_argument("--fullscreen", action="store_true", help="Run fullscreen")
    parser.add_argument(
        "--log-level", default="info", help="Log level: trace/debug/info/warn/error (default info)"
    )
    args = parser.parse_args()

    _log_map = {
        "trace": logging.DEBUG,
        "debug": logging.DEBUG,
        "info": logging.INFO,
        "warn": logging.WARNING,
        "error": logging.ERROR,
    }
    logging.basicConfig(level=_log_map.get(args.log_level.lower(), logging.INFO))

    rclpy.init()

    app = HoloscanSubscriberApp(headless=args.headless, fullscreen=args.fullscreen)
    app.run()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
