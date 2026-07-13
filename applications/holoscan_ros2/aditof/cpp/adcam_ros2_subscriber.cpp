/*
 * SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file aditof_subscriber.cpp
 *
 * @brief Holoscan ROS 2 subscriber for ADI ADTF3175 Time-of-Flight sensor.
 *
 * Subscribes to the three ROS 2 topics published by aditof_publisher and
 * displays them side-by-side using HolovizOp (mirrors adcam_player layout):
 *
 *   Left   (0-33%)   /aditof/depth_image   — 16-bit depth
 *   Center (33-66%)  /aditof/ab_image      — 16-bit active brightness
 *   Right  (66-100%) /aditof/conf_image    — 8-bit confidence
 */

#include <getopt.h>
#include <chrono>
#include <future>
#include <iostream>
#include <string>

#include <cuda.h>
#include <cuda_runtime.h>

#include <holoscan/holoscan.hpp>
#include <holoscan/operators/holoviz/holoviz.hpp>
#include <holoscan/ros2/bridge.hpp>
#include <holoscan/ros2/operators/subscriber.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace {

// ─────────────────────────────────────────────────────────────────────────────
//  AdiTofSubscriberOp
//  Subscribes to a single sensor_msgs/Image topic (16-bit or 8-bit),
//  transfers the data to GPU memory as a GXF tensor and emits it.
// ─────────────────────────────────────────────────────────────────────────────
class AdiTofSubscriberOp : public holoscan::ros2::ops::SubscriberOp<sensor_msgs::msg::Image> {
 public:
  HOLOSCAN_OPERATOR_FORWARD_ARGS_SUPER(AdiTofSubscriberOp,
                                       holoscan::ros2::ops::SubscriberOp<sensor_msgs::msg::Image>)

  AdiTofSubscriberOp() = default;

  void setup(holoscan::OperatorSpec& spec) override {
    spec.param(pool_, "pool", "Pool", "Allocator for output tensor");
    spec.param(tensor_name_,
               "tensor_name",
               "Tensor name",
               "Name used in the output GXF entity",
               std::string(""));
    spec.output<holoscan::gxf::Entity>("output");
    holoscan::ros2::ops::SubscriberOp<sensor_msgs::msg::Image>::setup(spec);
  }

  void compute(holoscan::InputContext& /*op_input*/, holoscan::OutputContext& op_output,
               holoscan::ExecutionContext& context) override {
    // Wait for a message with periodic timeout so we can handle:
    //   1. Subscriber started before publisher (publisher not yet sending)
    //   2. ROS 2 shutdown (rclcpp::ok() == false)
    //   3. Publisher temporarily stopped or restarted
    sensor_msgs::msg::Image message;
    while (rclcpp::ok()) {
      auto future = receive();
      auto status = future.wait_for(std::chrono::milliseconds(500));
      if (status == std::future_status::ready) {
        try {
          message = future.get();
          break;  // got a message — proceed
        } catch (const std::exception& e) {
          HOLOSCAN_LOG_WARN(
              "AdiTofSubscriberOp [{}]: receive() threw: {}", tensor_name_.get(), e.what());
          return;
        }
      }
      // Timeout — publisher not yet active, log once every ~5s then retry
      HOLOSCAN_LOG_DEBUG("AdiTofSubscriberOp [{}]: waiting for publisher on topic '{}'...",
                         tensor_name_.get(),
                         name());
    }

    if (!rclcpp::ok()) {
      HOLOSCAN_LOG_INFO("AdiTofSubscriberOp [{}]: ROS 2 shutdown detected, stopping.",
                        tensor_name_.get());
      return;
    }

    // Validate message fields before processing
    if (message.width == 0 || message.height == 0 || message.data.empty()) {
      HOLOSCAN_LOG_WARN("AdiTofSubscriberOp [{}]: received empty/invalid image, skipping.",
                        tensor_name_.get());
      return;
    }

    // Publisher sends rgb8 images — shape {H, W, 3}, dtype uint8_t.
    // The tensor_name_ must match the HolovizOp InputSpec name:
    //   "Depth", "ActiveBrightness", or "Conf"
    const int32_t h = static_cast<int32_t>(message.height);
    const int32_t w = static_cast<int32_t>(message.width);
    const size_t n_bytes = message.data.size();  // H * W * 3 bytes

    auto alloc_handle =
        nvidia::gxf::Handle<nvidia::gxf::Allocator>::Create(context.context(), pool_->gxf_cid());
    if (!alloc_handle) {
      HOLOSCAN_LOG_ERROR("AdiTofSubscriberOp: failed to create allocator");
      return;
    }

    // Always rgb8 — shape {H, W, 3}, uint8_t
    const nvidia::gxf::Shape shape{h, w, 3};

    auto out_entity =
        nvidia::gxf::CreateTensorMap(context.context(),
                                     alloc_handle.value(),
                                     {{tensor_name_.get().c_str(),
                                       nvidia::gxf::MemoryStorageType::kDevice,
                                       shape,
                                       nvidia::gxf::PrimitiveType::kUnsigned8,
                                       0,
                                       nvidia::gxf::ComputeTrivialStrides(shape, sizeof(uint8_t))}},
                                     false);

    if (!out_entity) {
      HOLOSCAN_LOG_ERROR("AdiTofSubscriberOp: tensor map creation failed");
      return;
    }

    auto maybe_tensor = out_entity.value().get<nvidia::gxf::Tensor>(tensor_name_.get().c_str());
    if (!maybe_tensor) {
      HOLOSCAN_LOG_ERROR("AdiTofSubscriberOp: tensor get failed");
      return;
    }

    cudaMemcpy(
        maybe_tensor.value()->pointer(), message.data.data(), n_bytes, cudaMemcpyHostToDevice);

    auto result = holoscan::gxf::Entity(std::move(out_entity.value()));
    op_output.emit(result, "output");
  }

 private:
  holoscan::Parameter<std::shared_ptr<holoscan::Allocator>> pool_;
  holoscan::Parameter<std::string> tensor_name_;
};

// ─────────────────────────────────────────────────────────────────────────────
//  HoloscanApplication
//  Three subscribers → HolovizOp side-by-side (mirrors adcam_player layout)
// ─────────────────────────────────────────────────────────────────────────────
class HoloscanApplication : public holoscan::Application {
 public:
  explicit HoloscanApplication(bool headless, bool fullscreen)
      : headless_(headless), fullscreen_(fullscreen) {}

  void compose() override {
    auto bridge = make_resource<holoscan::ros2::Bridge>("aditof_subscriber_resource",
                                                        "aditof_subscriber_node");

    // Allocator for all three planes — sized for largest possible frame
    // rgb8 = 3 bytes/pixel — sized for largest possible frame (1024×1024 × 3 channels)
    const size_t block_size = 1024 * 1024 * 3 * sizeof(uint8_t);
    auto pool = make_resource<holoscan::BlockMemoryPool>("pool", 1, block_size, 6);

    // ── Depth subscriber ──────────────────────────────────────────────────
    auto depth_sub = make_operator<AdiTofSubscriberOp>(
        "depth_subscriber",
        holoscan::Arg("ros2_bridge", bridge),
        holoscan::Arg("topic_name", std::string("aditof/depth_image")),
        holoscan::Arg("qos", holoscan::ros2::QoS(10)),
        holoscan::Arg("pool", pool),
        holoscan::Arg("tensor_name", std::string("Depth")));  // matches HolovizOp InputSpec name

    // ── AB subscriber ─────────────────────────────────────────────────────
    auto ab_sub = make_operator<AdiTofSubscriberOp>(
        "ab_subscriber",
        holoscan::Arg("ros2_bridge", bridge),
        holoscan::Arg("topic_name", std::string("aditof/ab_image")),
        holoscan::Arg("qos", holoscan::ros2::QoS(10)),
        holoscan::Arg("pool", pool),
        holoscan::Arg("tensor_name",
                      std::string("ActiveBrightness")));  // matches HolovizOp InputSpec name

    // ── Conf subscriber ───────────────────────────────────────────────────
    auto conf_sub = make_operator<AdiTofSubscriberOp>(
        "conf_subscriber",
        holoscan::Arg("ros2_bridge", bridge),
        holoscan::Arg("topic_name", std::string("aditof/conf_image")),
        holoscan::Arg("qos", holoscan::ros2::QoS(10)),
        holoscan::Arg("pool", pool),
        holoscan::Arg("tensor_name", std::string("Conf")));  // matches HolovizOp InputSpec name

    // ── HolovizOp — side-by-side layout matching adcam_player ─────────────
    holoscan::ops::HolovizOp::InputSpec depth_spec{"Depth",
                                                   holoscan::ops::HolovizOp::InputType::COLOR};
    depth_spec.views_ = {{0.0f, 0.0f, 0.33f, 1.0f}};

    holoscan::ops::HolovizOp::InputSpec ab_spec{"ActiveBrightness",
                                                holoscan::ops::HolovizOp::InputType::COLOR};
    ab_spec.views_ = {{0.33f, 0.0f, 0.33f, 1.0f}};

    holoscan::ops::HolovizOp::InputSpec conf_spec{"Conf",
                                                  holoscan::ops::HolovizOp::InputType::COLOR};
    conf_spec.views_ = {{0.66f, 0.0f, 0.34f, 1.0f}};

    auto visualizer = make_operator<holoscan::ops::HolovizOp>(
        "holoviz",
        holoscan::Arg("headless", headless_),
        holoscan::Arg("fullscreen", fullscreen_),
        holoscan::Arg("framebuffer_srgb", true),
        holoscan::Arg("window_title", std::string("ADI ToF Subscriber")),
        holoscan::Arg(
            "tensors",
            std::vector<holoscan::ops::HolovizOp::InputSpec>{depth_spec, ab_spec, conf_spec}));

    // ── Data flow ──────────────────────────────────────────────────────────
    add_flow(depth_sub, visualizer, {{"output", "receivers"}});
    add_flow(ab_sub, visualizer, {{"output", "receivers"}});
    add_flow(conf_sub, visualizer, {{"output", "receivers"}});
  }

 private:
  bool headless_;
  bool fullscreen_;
};

}  // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  bool headless = false;
  bool fullscreen = false;

  const struct option long_options[] = {{"headless", no_argument, nullptr, 0},
                                        {"fullscreen", no_argument, nullptr, 0},
                                        {"help", no_argument, nullptr, 'h'},
                                        {nullptr, 0, nullptr, 0}};

  while (true) {
    int idx = 0;
    int c = getopt_long(argc, argv, "h", long_options, &idx);
    if (c == -1)
      break;
    if (c == 0) {
      std::string name(long_options[idx].name);
      if (name == "headless")
        headless = true;
      if (name == "fullscreen")
        fullscreen = true;
    } else if (c == 'h') {
      std::cout << "Usage: holoscan_ros2_aditof_subscriber [options]\n"
                << "  --headless     Run without display\n"
                << "  --fullscreen   Run fullscreen\n"
                << "\nSubscribes to:\n"
                << "  /aditof/depth_image   (16UC1)\n"
                << "  /aditof/ab_image      (16UC1)\n"
                << "  /aditof/conf_image    (8UC1)\n";
      return EXIT_SUCCESS;
    }
  }

  // Auto-enable headless mode when no display is available (Docker without X11 forwarding)
  if (!headless && std::getenv("DISPLAY") == nullptr && std::getenv("WAYLAND_DISPLAY") == nullptr) {
    HOLOSCAN_LOG_INFO(
        "No DISPLAY or WAYLAND_DISPLAY detected — enabling headless (offscreen) mode.");
    headless = true;
  }

  HoloscanApplication app(headless, fullscreen);
  app.run();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
