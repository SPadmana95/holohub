# Holoscan ROS 2 ADI ToF (ADTF3175) Publisher / Subscriber

## Overview

This application adds **ROS 2 publishing and subscribing** to the ADI ADTF3175 Time-of-Flight sensor pipeline running on the **Holoscan Sensor Bridge (HSB)**.

It extends `holoscan-sensor-bridge/examples/aditof/cpp/adcam_player` by replacing the HolovizOp display with ROS 2 topic publishers, and provides a separate subscriber that receives those topics and visualizes them with HolovizOp.

---

## File Structure

```
applications/holoscan_ros2/aditof/
├── CMakeLists.txt              # Top-level build — C++ only
├── README.md                   # This file
└── cpp/
    ├── CMakeLists.txt          # C++ build config
    ├── metadata.json           # Application metadata with publisher/subscriber modes
    ├── aditof_publisher.cpp    # Publisher: HSB → AdcamUnpackOp → ROS 2 topics
    └── aditof_subscriber.cpp   # Subscriber: ROS 2 topics → HolovizOp
```

---

## Prerequisites

| Requirement | Detail |
|---|---|
| Host Device | NVIDIA AGX Thor |
| JetPack | 7.0 (LinuxReceiverOp) or 7.1+ (RoceReceiverOp) |
| Holoscan SDK | 3.9.0 |
| Holoscan Sensor Bridge | 2.5.0 |
| HSB hardware | Hololink FPGA board at `192.168.0.2` |
| Sensor | ADI ADTF3175 (with ADSD3100 / ADSD3066 imager) connected to HSB |
| ROS 2 | Jazzy |

---

## Published Topics (Publisher Mode)

| Topic | Message Type | Content |
|---|---|---|
| `/aditof/depth_image` | `sensor_msgs/Image` (16UC1) | 16-bit depth per pixel (mm) |
| `/aditof/ab_image` | `sensor_msgs/Image` (16UC1) | 16-bit active brightness (IR intensity) |
| `/aditof/conf_image` | `sensor_msgs/Image` (8UC1) | 8-bit confidence score per pixel |

---

## Pipeline

### Publisher pipeline

```
ADI ADTF3175 ToF sensor
    ↓ MIPI CSI-2
ADSD3500 depth processor (depth + AB + confidence)
    ↓ MIPI CSI-2
HSB FPGA (Hololink board, 192.168.0.2)
    ↓ 10GigE (Linux network — no InfiniBand required on JP 7.0)
LinuxReceiverOp  (or RoceReceiverOp on JP 7.1+)
    ↓
CsiToBayerOp
    ↓
AdcamUnpackOp  (unpacks ADI 5-byte/pixel format → depth, AB, conf planes)
    ↓
PublisherOp<sensor_msgs::msg::Image> × 3
    ↓
/aditof/depth_image   /aditof/ab_image   /aditof/conf_image
```

### Subscriber pipeline

```
/aditof/depth_image   /aditof/ab_image   /aditof/conf_image
    ↓
SubscriberOp<sensor_msgs::msg::Image> × 3
    ↓
HolovizOp (side-by-side: Depth | AB | Confidence)
```

---

## Building

```sh
# Build container (from holohub root)
./holohub build-container aditof --language cpp \
  --base-img nvcr.io/nvidia/clara-holoscan/holoscan:v3.9.0-cuda13 \
  --build-args="--no-cache"

# Launch container
./holohub run-container aditof --language cpp

# Inside container — build
./holohub build aditof --language cpp
```

---

## Running

### Terminal 1 — Publisher

```sh
./holohub run aditof publisher --language cpp
```

Options:

| Option | Default | Description |
|---|---|---|
| `--hololink` | `192.168.0.2` | IP of the Hololink board |
| `--ibv-name` | auto | IBV device (leave empty for Linux receiver) |
| `--headless` | — | Run without display |

### Terminal 2 — Subscriber (HolovizOp)

```sh
./holohub run aditof subscriber --language cpp
```

### Alternative: RViz2

```sh
source /opt/ros/jazzy/setup.bash
rviz2
# Add Image display → Topic: /aditof/depth_image
# Add Image display → Topic: /aditof/ab_image
# Add Image display → Topic: /aditof/conf_image
```

---

## Key Differences from `vb1940`

| Aspect | `vb1940` | `aditof` |
|---|---|---|
| Sensor | VB1940 Eagle RGB camera | ADI ADTF3175 ToF sensor |
| Message type | `sensor_msgs/Image` (rgb8) | `sensor_msgs/Image` (16UC1 depth/AB, 8UC1 conf) |
| Published topics | 1 (`vb1940/image`) | 3 (`depth_image`, `ab_image`, `conf_image`) |
| Unpack step | CUDA 16→8 bit conversion | `AdcamUnpackOp` (ADI 5-byte/pixel format) |
| Receiver | `RoceReceiverOp` (IBV required) | `LinuxReceiverOp` (no IBV, works on JP 7.0) |
| InfiniBand required | Yes (JP 7.1+ for Thor) | **No** (JP 7.0 compatible) |
