# Holoscan ROS 2 ADI ToF (ADTF3175) Publisher / Subscriber

## Overview

This application adds **ROS 2 publishing and subscribing** to the ADI ADTF3175 Time-of-Flight sensor pipeline running on the **Holoscan Sensor Bridge (HSB)**.

It extends `holoscan-sensor-bridge/examples/aditof/cpp/adcam_player` by replacing the HolovizOp display with ROS 2 topic publishers, and provides a separate subscriber that receives those topics and visualizes them with HolovizOp.

---

## File Structure

```
applications/holoscan_ros2/aditof/
├── CMakeLists.txt          # Top-level build — C++ only
├── Dockerfile              # Shared container for C++ and Python
├── adi_manifest.yaml       # Firmware manifest for ADI ADTF3175 sensor
├── README.md               # This file
├── cpp/
│   ├── CMakeLists.txt                      # C++ build config
│   ├── metadata.json                       # Application metadata with publisher/subscriber 
│   ├── adcam_ros2_publisher.cpp            # Publisher: HSB → AdcamUnpackOp → ROS 2 topics
│   ├── adcam_ros2_subscriber.cpp           # Subscriber: ROS 2 topics → HolovizOp
│   ├── adcam_lib.cpp / .hpp                # ADCAM sensor library
│   ├── adcam_unpack_op.cpp / .cu / .hpp    # ADI 5-byte/pixel unpack (CUDA)
│   ├── adsd3500_flash.cpp / .hpp           # Firmware flash logic
│   ├── programmer.cpp / .hpp               # Manifest-based firmware programmer
│   ├── compute_crc.cpp / .hpp              # CRC-32 computation
│   └── crc_table.cpp                       # CRC lookup table
└── python/
    ├── metadata.json               # Application metadata with publisher/subscriber modes
    ├── adcam_ros2_publisher.py     # Publisher: HSB → ADTFUnpackOp → ROS 2 topics
    ├── adcam_ros2_subscriber.py    # Subscriber: ROS 2 topics → HolovizOp
    ├── adcam_unpack_op.py          # ADTFUnpackOp: unpack frames and colorize
    └── adcam.py                    # ADI ADTF3175 sensor driver
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

> **Note:** The same container image and build step serve both C++ and Python.
> `--language cpp` is used throughout because both language variants share the same
> Dockerfile and there is nothing to compile for Python.

Build the container from the HoloHub root. It is used for both C++ and Python.

```sh
./holohub build-container aditof --language cpp \
  --base-img nvcr.io/nvidia/clara-holoscan/holoscan:v4.6.0-cuda13
```

Launch the container.

```sh
./holohub run-container aditof --language cpp
```

Inside the container, build the C++ binaries. Python needs no build step.

```sh
./holohub build aditof --language cpp
```

---

## Running

> **Note:** Use `--run-args` to pass arguments to the application executable.
> This is the holohub-correct way — the `--` separator is not supported by the holohub CLI.

### Terminal 1 — Publisher

#### C++

Print publisher options and exit.

```sh
./holohub run aditof publisher --language cpp
```

Print the Imager available capture mode map and exit.

```sh
./holohub run aditof publisher --language cpp --run-args="--getModes 1"
```

Set capture mode and start streaming.

```sh
./holohub run aditof publisher --language cpp --run-args="--captureMode 3"
```

Update sensor firmware using the manifest file. `adi_manifest.yaml` is in the
application directory; the command uses its absolute container path.

```sh
./holohub run aditof publisher --language cpp --run-args="--firmwareUpdate /workspace/holohub/applications/holoscan_ros2/aditof/adi_manifest.yaml"
```

**All `--run-args` options (C++):**

| Option | Default | Description |
|---|---|---|
| `--hololink <ip>` | `192.168.0.2` | IP address of the Hololink board |
| `--captureMode <0-9>` | required | ADI capture mode for streaming |
| `--resetPin <0-31>` | `0` | GPIO pin used for sensor reset |
| `--firmwareUpdate <yaml>` | — | Update sensor firmware using manifest file |
| `--getModes <0\|1>` | `0` | Print the Imager available capture mode map and exit when set to `1` |
| `--frame-limit <n>` | `0` (unlimited) | Stop after N frames |
| `--ibv-name <dev>` | auto | IBV device name (empty = LinuxReceiverOp) |
| `--ibv-port <n>` | `1` | IBV port |

#### Python

Print the Imager available capture mode map and exit.

```sh
./holohub run aditof publisher --language python --run-args="--getModes 1"
```

Set capture mode and start streaming.

```sh
./holohub run aditof publisher --language python --run-args="--captureMode 3"
```

Update sensor firmware using the manifest file.

```sh
./holohub run aditof publisher --language python --run-args="--firmwareUpdate /workspace/holohub/applications/holoscan_ros2/aditof/adi_manifest.yaml"
```

**All `--run-args` options (Python):**

| Option | Default | Description |
|---|---|---|
| `--hololink <ip>` | `192.168.0.2` | IP address of the Hololink board |
| `--captureMode <0-9>` | required | ADI capture mode for streaming |
| `--resetPin <0-31>` | `0` | GPIO pin used for sensor reset |
| `--firmwareUpdate <yaml>` | — | Update sensor firmware using manifest file |
| `--getModes <0\|1>` | `0` | Print the Imager available capture mode map and exit when set to `1` |
| `--force` | — | Allow firmware downgrade (use with `--firmwareUpdate`) |
| `--frame-limit <n>` | `0` (unlimited) | Stop after N frames |
| `--ibv-name <dev>` | auto | IBV device name (empty = LinuxReceiverOp) |
| `--ibv-port <n>` | `1` | IBV port |
| `--log-level <level>` | `info` | Log level: trace/debug/info/warn/error |

### Terminal 2 — Subscriber (HolovizOp)

#### C++

```sh
./holohub run aditof subscriber --language cpp
```

#### Python

Run without a display.

```sh
./holohub run aditof subscriber --language python
./holohub run aditof subscriber --language python --run-args="--headless"
```

### Alternative: RViz2

#### Installation

Install RViz2.

```sh
sudo apt update
sudo apt install -y ros-jazzy-rviz2
```

Verify the installation.

```sh
source /opt/ros/jazzy/setup.bash
rviz2 --version
```

#### Step-by-step: Add topics to RViz2

**Step 1 — Start the publisher** (Terminal 1):
```sh
./holohub run aditof publisher --language cpp
```

**Step 2 — Launch RViz2** (Terminal 2):
```sh
source /opt/ros/jazzy/setup.bash
rviz2
```

**Step 3 — Fix QoS (required):**
The publisher uses `RELIABLE` QoS but RViz2 defaults to `Best Effort`. In each
Image display panel → click the **Topic** dropdown → change **Reliability Policy**
from `Best Effort` to `Reliable`.

**Step 4 — Add Depth Image:**
1. Click **Add** (bottom-left) → **By topic** tab
2. Find `/aditof/depth_image` → select **Image** → click **OK**
3. Expand the display → **Topic** → set **Reliability Policy** to `Reliable`

**Step 5 — Add AB Image:**
1. Click **Add** → **By topic**
2. Find `/aditof/ab_image` → select **Image** → click **OK**
3. Set **Reliability Policy** to `Reliable`

**Step 6 — Add Confidence Image:**
1. Click **Add** → **By topic**
2. Find `/aditof/conf_image` → select **Image** → click **OK**
3. Set **Reliability Policy** to `Reliable`

#### Launch RViz2 with all three topics at once

```sh
source /opt/ros/jazzy/setup.bash
rviz2 -d - << 'EOF'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Displays:
    - Class: rviz_common/Image
      Name: Depth
      Topic:
        Value: /aditof/depth_image
        Durability Policy: Volatile
        Reliability Policy: Reliable
    - Class: rviz_common/Image
      Name: AB
      Topic:
        Value: /aditof/ab_image
        Durability Policy: Volatile
        Reliability Policy: Reliable
    - Class: rviz_common/Image
      Name: Confidence
      Topic:
        Value: /aditof/conf_image
        Durability Policy: Volatile
        Reliability Policy: Reliable
EOF
```
