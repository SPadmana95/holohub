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
 * @file aditof_publisher.cpp
 *
 * @brief Holoscan ROS 2 publisher for ADI ADTF3175 Time-of-Flight sensor.
 *
 * Replaces HolovizOp from adcam_player.cpp with three ROS 2 PublisherOp
 * operators that publish Depth, Active Brightness (AB) and Confidence images
 * to separate topics.
 *
 * Pipeline:
 *   HSB (LinuxReceiverOp / RoceReceiverOp)
 *     → CsiToBayerOp
 *     → ADTFUnpackOp  (5-byte/pixel → depth, AB, conf planes)
 *     → AdiTofPublisherOp
 *         ├─ PublisherOp<Image>  →  /aditof/depth_image   (16UC1)
 *         ├─ PublisherOp<Image>  →  /aditof/ab_image      (16UC1)
 *         └─ PublisherOp<Image>  →  /aditof/conf_image    (8UC1)
 */

#include <getopt.h>
#include <iostream>
#include <string>

#include <cuda.h>
#include <cuda_runtime.h>

#include <hololink/common/cuda_helper.hpp>
#include <hololink/common/tools.hpp>
#include <hololink/core/data_channel.hpp>
#include <hololink/core/enumerator.hpp>
#include <hololink/core/hololink.hpp>
#include <hololink/core/logging.hpp>
#include <hololink/operators/csi_to_bayer/csi_to_bayer.hpp>
#include <hololink/operators/image_processor/image_processor.hpp>
#include <hololink/operators/linux_receiver/linux_receiver_op.hpp>
#include <hololink/operators/roce_receiver/roce_receiver_op.hpp>

#include "adcam_lib.hpp"
#include "adcam_unpack_op.hpp"
#include "programmer.hpp"

#include <holoscan/holoscan.hpp>
#include <holoscan/ros2/bridge.hpp>
#include <holoscan/ros2/operators/publisher.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace {

// ─────────────────────────────────────────────────────────────────────────────
//  AdiTofPublisherOp
//  Receives the 3-plane GXF entity from ADTFUnpackOp (Depth/AB/Conf) and
//  publishes each plane as a separate sensor_msgs/Image topic.
// ─────────────────────────────────────────────────────────────────────────────
class AdiTofPublisherOp : public holoscan::Operator {
 public:
  HOLOSCAN_OPERATOR_FORWARD_ARGS(AdiTofPublisherOp)

  AdiTofPublisherOp() = default;

  void setup(holoscan::OperatorSpec& spec) override {
    spec.input<holoscan::gxf::Entity>("input");
    spec.param(ros2_bridge_, "ros2_bridge", "ROS2 Bridge", "Shared ROS2 bridge");
    spec.param(frame_id_, "frame_id", "Frame ID", "TF frame id", std::string("aditof_optical"));
  }

  void initialize() override {
    holoscan::Operator::initialize();
    auto bridge = ros2_bridge_.get();

    depth_pub_ = bridge->create_publisher<sensor_msgs::msg::Image>(
        "aditof/depth_image", rclcpp::QoS(10));
    ab_pub_ = bridge->create_publisher<sensor_msgs::msg::Image>(
        "aditof/ab_image", rclcpp::QoS(10));
    conf_pub_ = bridge->create_publisher<sensor_msgs::msg::Image>(
        "aditof/conf_image", rclcpp::QoS(10));
  }

  void compute(holoscan::InputContext& op_input,
               holoscan::OutputContext& /*op_output*/,
               holoscan::ExecutionContext& /*context*/) override {
    auto entity = op_input.receive<holoscan::gxf::Entity>("input").value();
    auto& gxf_entity = static_cast<nvidia::gxf::Entity&>(entity);
    auto stamp = rclcpp::Clock().now();

    // ADTFUnpackOp outputs 3 SEPARATE named tensors, each shape {H, W, 3} uint8_t RGB:
    //   "Depth"            — Jet-colormap RGB visualization of depth
    //   "ActiveBrightness" — Grayscale-as-RGB visualization of AB (normalized to 4096)
    //   "Conf"             — Grayscale-as-RGB visualization of confidence (normalized to 255)
    publish_tensor(gxf_entity, "Depth",            stamp, depth_pub_);
    publish_tensor(gxf_entity, "ActiveBrightness", stamp, ab_pub_);
    publish_tensor(gxf_entity, "Conf",             stamp, conf_pub_);
  }

 private:
  // Use Bridge::Publisher — bridge->create_publisher<T>() returns Bridge::Publisher<T>::SharedPtr,
  // NOT rclcpp::Publisher<T>::SharedPtr (they are different wrapper types)
  using Publisher = typename holoscan::ros2::Bridge::Publisher<sensor_msgs::msg::Image>::SharedPtr;

  void publish_tensor(nvidia::gxf::Entity& entity,
                      const std::string& tensor_name,
                      const rclcpp::Time& stamp,
                      Publisher& pub) {
    auto maybe_tensor = entity.get<nvidia::gxf::Tensor>(tensor_name.c_str());
    if (!maybe_tensor) {
      HOLOSCAN_LOG_ERROR("AdiTofPublisherOp: tensor '{}' not found", tensor_name);
      return;
    }
    auto tensor_handle = maybe_tensor.value();  // Handle<Tensor> — not a raw pointer

    // Shape: {height, width, 3} — uint8_t RGB (colorized by ADTFUnpackOp)
    const int32_t height   = tensor_handle->shape().dimension(0);
    const int32_t width    = tensor_handle->shape().dimension(1);
    const size_t  n_bytes  = static_cast<size_t>(height) * width * 3 * sizeof(uint8_t);

    sensor_msgs::msg::Image msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = frame_id_.get();
    msg.height   = static_cast<uint32_t>(height);
    msg.width    = static_cast<uint32_t>(width);
    msg.encoding = "rgb8";
    msg.step     = static_cast<uint32_t>(width * 3);
    msg.data.resize(n_bytes);

    // GPU → CPU transfer
    cudaMemcpy(msg.data.data(), tensor_handle->pointer(), n_bytes, cudaMemcpyDeviceToHost);
    pub->publish(msg);
  }

  holoscan::Parameter<std::shared_ptr<holoscan::ros2::Bridge>> ros2_bridge_;
  holoscan::Parameter<std::string> frame_id_;

  Publisher depth_pub_;
  Publisher ab_pub_;
  Publisher conf_pub_;
};

// ─────────────────────────────────────────────────────────────────────────────
//  HoloscanApplication
//  Replicates adcam_player pipeline but replaces HolovizOp with
//  AdiTofPublisherOp that publishes to ROS 2 topics.
// ─────────────────────────────────────────────────────────────────────────────
class HoloscanApplication : public holoscan::Application {
 public:
  explicit HoloscanApplication(
      CUcontext cuda_context,
      int cuda_device_ordinal,
      std::shared_ptr<hololink::DataChannel> hololink_channel,
      const std::string& ibv_name, uint32_t ibv_port,
      std::shared_ptr<hololink::sensors::Adcam> adcam_inst,
      int64_t frame_limit)
      : cuda_context_(cuda_context),
        cuda_device_ordinal_(cuda_device_ordinal),
        hololink_channel_(hololink_channel),
        ibv_name_(ibv_name),
        ibv_port_(ibv_port),
        adcam_inst_(adcam_inst),
        frame_limit_(frame_limit) {}

  void compose() override {
    // ── Condition ──────────────────────────────────────────────────────────
    std::shared_ptr<holoscan::Condition> condition;
    if (frame_limit_) {
      condition = make_condition<holoscan::CountCondition>("count", frame_limit_);
    } else {
      condition = make_condition<holoscan::BooleanCondition>("ok", true);
    }

    // ── Memory pool for CSI → Bayer ────────────────────────────────────────
    auto csi_pool = make_resource<holoscan::BlockMemoryPool>(
        "csi_to_bayer_pool", 1,
        adcam_inst_->get_width() * adcam_inst_->get_height() * sizeof(uint16_t),
        2);

    // ── CSI → Bayer operator ───────────────────────────────────────────────
    auto csi_to_bayer = make_operator<hololink::operators::CsiToBayerOp>(
        "csi_to_bayer",
        holoscan::Arg("allocator", csi_pool),
        holoscan::Arg("cuda_device_ordinal", cuda_device_ordinal_));

    std::shared_ptr<hololink::csi::CsiConverter> csi_converter = csi_to_bayer;

    // ── Probe and configure sensor ─────────────────────────────────────────
    if (!adcam_inst_->probe_adcam_adtf3175()) {
      HOLOSCAN_LOG_ERROR("ADTF3175 NOT found — check connections");
      throw std::runtime_error("ADTF3175 not found");
    }
    adcam_inst_->configure_converter(csi_to_bayer);
    adcam_inst_->set_mipi(adcam_inst_);
    adcam_inst_->set_mode();

    const size_t frame_size = csi_to_bayer->get_csi_length();
    HOLOSCAN_LOG_INFO("ADI ToF publisher: frame_size={}", frame_size);

    // ── Receiver (Linux or ROCE) ───────────────────────────────────────────
    std::shared_ptr<holoscan::Operator> receiver;
    if (!ibv_name_.empty()) {
      receiver = make_operator<hololink::operators::RoceReceiverOp>(
          "receiver", condition,
          holoscan::Arg("frame_size", frame_size),
          holoscan::Arg("frame_context", cuda_context_),
          holoscan::Arg("ibv_name", ibv_name_),
          holoscan::Arg("ibv_port", ibv_port_),
          holoscan::Arg("hololink_channel", hololink_channel_.get()),
          holoscan::Arg("device_start", std::function<void()>([this] { adcam_inst_->start(); })),
          holoscan::Arg("device_stop",  std::function<void()>([this] { adcam_inst_->stop();  })));
    } else {
      receiver = make_operator<hololink::operators::LinuxReceiverOp>(
          "receiver", condition,
          holoscan::Arg("frame_size", frame_size),
          holoscan::Arg("frame_context", cuda_context_),
          holoscan::Arg("hololink_channel", hololink_channel_.get()),
          holoscan::Arg("device_start", std::function<void()>([this] { adcam_inst_->start(); })),
          holoscan::Arg("device_stop",  std::function<void()>([this] { adcam_inst_->stop();  })));
    }

    // ── Memory pool for ADTFUnpackOp ───────────────────────────────────────
    const size_t pool_block_size =
        adcam_inst_->get_width() * adcam_inst_->get_height() * sizeof(uint16_t);
    auto adtf_pool = make_resource<holoscan::BlockMemoryPool>(
        "ADTF_output_pool", 1, pool_block_size, 8);

    // ── ADTFUnpackOp — 5-byte/pixel → Depth / AB / Conf ───────────────────
    auto adtf_unpack = make_operator<hololink::operators::ADTFUnpackOp>(
        "ADIToF_data",
        holoscan::Arg("num_planes", 3),
        holoscan::Arg("width",  static_cast<int>(adcam_inst_->get_pixel_width())),
        holoscan::Arg("height", static_cast<int>(adcam_inst_->get_pixel_height())),
        holoscan::Arg("allocator", adtf_pool),
        holoscan::Arg("in_tensor_name",  std::string("")),
        holoscan::Arg("out_tensor_name", std::string("output")));

    // ── ROS 2 Bridge and Publisher ─────────────────────────────────────────
    auto ros2_bridge = make_resource<holoscan::ros2::Bridge>(
        "aditof_publisher_resource", "aditof_publisher_node");

    auto ros2_publisher = make_operator<AdiTofPublisherOp>(
        "aditof_publisher",
        holoscan::Arg("ros2_bridge", ros2_bridge),
        holoscan::Arg("frame_id", std::string("aditof_optical")));

    // ── Data flow ──────────────────────────────────────────────────────────
    add_flow(receiver,     csi_to_bayer, {{"output", "input"}});
    add_flow(csi_to_bayer, adtf_unpack,  {{"output", "input"}});
    add_flow(adtf_unpack,  ros2_publisher, {{"output", "input"}});
  }

 private:
  const CUcontext cuda_context_;
  const int cuda_device_ordinal_;
  std::shared_ptr<hololink::DataChannel> hololink_channel_;
  const std::string ibv_name_;
  const uint32_t ibv_port_;
  std::shared_ptr<hololink::sensors::Adcam> adcam_inst_;
  const int64_t frame_limit_;
};

}  // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // ── Defaults ───────────────────────────────────────────────────────────────
  int32_t     adcam_mode  = 6;
  int64_t     frame_limit = 0;
  std::string hololink_ip = "192.168.0.2";
  std::string ibv_name;
  uint32_t    ibv_port = 1;
  int32_t     reset_pin = 0;
  bool        do_reset  = false;
  std::string firmware_manifest;

  // Auto-detect InfiniBand device (empty = use LinuxReceiverOp)
  try {
    auto devices = hololink::infiniband_devices();
    ibv_name = devices.empty() ? "" : devices[0];
  } catch (...) {
    ibv_name = "";
  }

  // ── CLI parsing ────────────────────────────────────────────────────────────
  const struct option long_options[] = {
      {"captureMode",    required_argument, nullptr, 0},
      {"frame-limit",    required_argument, nullptr, 0},
      {"hololink",       required_argument, nullptr, 0},
      {"ibv-name",       required_argument, nullptr, 0},
      {"ibv-port",       required_argument, nullptr, 0},
      {"resetAdcam",     required_argument, nullptr, 0},
      {"resetPin",       required_argument, nullptr, 0},
      {"firmwareUpdate", required_argument, nullptr, 0},
      {"help",           no_argument,       nullptr, 'h'},
      {nullptr, 0, nullptr, 0}};

  while (true) {
    int idx = 0;
    int c   = getopt_long(argc, argv, "h", long_options, &idx);
    if (c == -1) break;
    const std::string arg(optarg ? optarg : "");
    if (c == 0) {
      const std::string name(long_options[idx].name);
      if (name == "captureMode")    adcam_mode  = std::stoi(arg);
      else if (name == "frame-limit")    frame_limit = std::stoll(arg);
      else if (name == "hololink")       hololink_ip = arg;
      else if (name == "ibv-name")       ibv_name    = arg;
      else if (name == "ibv-port")       ibv_port    = std::stoul(arg);
      else if (name == "resetAdcam")     do_reset    = std::stoi(arg) != 0;
      else if (name == "resetPin")       reset_pin   = std::stoi(arg);
      else if (name == "firmwareUpdate") firmware_manifest = arg;
    } else if (c == 'h') {
      std::cout
          << "Usage: holoscan_ros2_aditof_publisher [options]\n"
          << "  --captureMode <0-9>       ADI capture mode (default 6)\n"
          << "  --hololink <ip>           Hololink board IP (default 192.168.0.2)\n"
          << "  --ibv-name <dev>          IBV device (empty = LinuxReceiverOp)\n"
          << "  --ibv-port <n>            IBV port (default 1)\n"
          << "  --frame-limit <n>         Stop after N frames (0=unlimited)\n"
          << "  --resetAdcam <0/1>        Reset ADCAM on startup\n"
          << "  --resetPin <0-31>         Reset pin number\n"
          << "  --firmwareUpdate <yaml>   Update firmware using manifest\n";
      return EXIT_SUCCESS;
    }
  }

  // ── Initialise CUDA ────────────────────────────────────────────────────────
  CudaCheck(cuInit(0));
  CUdevice cu_device;
  CudaCheck(cuDeviceGet(&cu_device, 0));
  CUcontext cu_context;
  CudaCheck(cuDevicePrimaryCtxRetain(&cu_context, cu_device));

  // ── Connect to Hololink board ──────────────────────────────────────────────
  auto channel_metadata = hololink::Enumerator::find_channel(hololink_ip);
  auto hololink_channel = std::make_shared<hololink::DataChannel>(channel_metadata);

  // ── Create ADCAM instance ──────────────────────────────────────────────────
  auto adcam_inst = std::make_shared<hololink::sensors::Adcam>(
      hololink_channel, hololink::CAM_I2C_BUS,
      channel_metadata, adcam_mode, reset_pin);

  // ── Start Hololink ─────────────────────────────────────────────────────────
  auto hololink = hololink_channel->hololink();
  hololink->start();

  if (do_reset) {
    adcam_inst->adcam_reset_power_on();
  }

  if (!firmware_manifest.empty()) {
    hololink::Programmer::Args args;
    args.manifest   = firmware_manifest;
    args.hololink_ip = hololink_ip;
    hololink::Programmer programmer(args, args.manifest);
    programmer.fetch_manifest("hololink");
    programmer.check_eula();
    programmer.check_images();
    programmer.program_and_verify_images(hololink, adcam_inst);
    hololink->stop();
    CudaCheck(cuDevicePrimaryCtxRelease(cu_device));
    return EXIT_SUCCESS;
  }

  // ── Run application ────────────────────────────────────────────────────────
  HoloscanApplication app(cu_context, 0, hololink_channel,
                           ibv_name, ibv_port, adcam_inst, frame_limit);
  app.run();

  hololink->stop();
  CudaCheck(cuDevicePrimaryCtxRelease(cu_device));
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
