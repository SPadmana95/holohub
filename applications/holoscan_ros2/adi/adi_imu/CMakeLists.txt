# adi_imu: ROS2 publisher for ADI ADIS16505-2/ADIS16607-2 IMU sensors.
# The native operator and its Python bindings are built by the adi_imu
# operator (see operators/holoscan_ros2/adi/adi_imu/), pulled in via the
# DEPENDS OPERATORS clause in the parent CMakeLists.txt; this application
# directory is a Python-only ROS 2 script.
cmake_minimum_required(VERSION 3.20)
project(adi_imu LANGUAGES CXX)

find_package(rclcpp REQUIRED HINTS /opt/ros/jazzy)
find_package(sensor_msgs REQUIRED)
find_package(holoscan 3.0 REQUIRED CONFIG
			 PATHS "/opt/nvidia/holoscan" "/workspace/holoscan-sdk/install")

set(HOLOLINK_ROOT "/deps/holoscan-sensor-bridge")
set(HOLOLINK_BUILD_DIR "${HOLOLINK_ROOT}/build")
set(HOLOLINK_INCLUDE_DIR "${HOLOLINK_ROOT}/src")
link_directories(
		"${HOLOLINK_BUILD_DIR}/src/hololink/core"
		"${HOLOLINK_BUILD_DIR}/src/hololink/common")

if(NOT TARGET hololink::hololink)
	add_library(hololink STATIC IMPORTED)
	set_target_properties(hololink PROPERTIES
			IMPORTED_LOCATION "${HOLOLINK_BUILD_DIR}/src/hololink/common/libhololink.a")
	add_library(hololink::hololink ALIAS hololink)
endif()

add_executable(adi_imu_ros2_publisher cpp/adi_imu_ros2_publisher.cpp)
target_link_libraries(adi_imu_ros2_publisher PRIVATE
	holoscan::core
	holoscan::ops::adi_imu
	${rclcpp_LIBRARIES}
	${sensor_msgs_LIBRARIES}
	hololink::hololink
	hololink_core
	fmt)
target_include_directories(adi_imu_ros2_publisher PRIVATE
	${rclcpp_INCLUDE_DIRS}
	${sensor_msgs_INCLUDE_DIRS}
	${HOLOLINK_INCLUDE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/../../../../../operators/holoscan_ros2/adi/adi_imu/cpp)

add_executable(adi_imu_ros2_subscriber cpp/adi_imu_ros2_subscriber.cpp)
target_link_libraries(adi_imu_ros2_subscriber PRIVATE ${rclcpp_LIBRARIES} ${sensor_msgs_LIBRARIES})
target_include_directories(adi_imu_ros2_subscriber PRIVATE ${rclcpp_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS})

install(TARGETS adi_imu_ros2_publisher adi_imu_ros2_subscriber
	DESTINATION ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY python/
	DESTINATION ${CMAKE_INSTALL_DATADIR}/holohub/adi/adi_imu/python
	FILES_MATCHING PATTERN "*.py")
