//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <iostream>
#include <chrono>
#include <stdexcept>
#include <vector>
#include <csignal>
#include <thread>

#include "include/livox_ros_driver2.h"
#include "include/ros_headers.h"
#include "driver_node.h"
#include "lddc.h"
#include "lds_lidar.h"

using namespace livox_ros;

#ifdef BUILDING_ROS1
int main(int argc, char **argv) {
  /** Ros related */
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::init(argc, argv, "livox_lidar_publisher");

  // ros::NodeHandle livox_node;
  livox_ros::DriverNode livox_node;

  DRIVER_INFO(livox_node, "Livox Ros Driver2 Version: %s", LIVOX_ROS_DRIVER2_VERSION_STRING);

  /** Init default system parameter */
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq  = 10.0; /* Hz */
  int output_type      = kOutputToRos;
  std::string frame_id = "livox_frame";
  bool lidar_bag = true;
  bool imu_bag   = false;

  livox_node.GetNode().getParam("xfer_format", xfer_format);
  livox_node.GetNode().getParam("multi_topic", multi_topic);
  livox_node.GetNode().getParam("data_src", data_src);
  livox_node.GetNode().getParam("publish_freq", publish_freq);
  livox_node.GetNode().getParam("output_data_type", output_type);
  livox_node.GetNode().getParam("frame_id", frame_id);
  livox_node.GetNode().getParam("enable_lidar_bag", lidar_bag);
  livox_node.GetNode().getParam("enable_imu_bag", imu_bag);

  printf("data source:%u.\n", data_src);

  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.5) {
    publish_freq = 0.5;
  } else {
    publish_freq = publish_freq;
  }

  livox_node.future_ = livox_node.exit_signal_.get_future();

  /** Lidar data distribute control and lidar data source set */
  livox_node.lddc_ptr_ = std::make_unique<Lddc>(xfer_format, multi_topic, data_src, output_type,
                        publish_freq, frame_id, lidar_bag, imu_bag);
  livox_node.lddc_ptr_->SetRosNode(&livox_node);

  if (data_src == kSourceRawLidar) {
    DRIVER_INFO(livox_node, "Data Source is raw lidar.");

    std::string user_config_path;
    livox_node.getParam("user_config_path", user_config_path);
    DRIVER_INFO(livox_node, "Config file : %s", user_config_path.c_str());

    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);
    livox_node.lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

    if ((read_lidar->InitLdsLidar(user_config_path))) {
      DRIVER_INFO(livox_node, "Init lds lidar successfully!");
    } else {
      DRIVER_ERROR(livox_node, "Init lds lidar failed!");
    }
  } else {
    DRIVER_ERROR(livox_node, "Invalid data src (%d), please check the launch file", data_src);
  }

  livox_node.pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, &livox_node);
  livox_node.imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, &livox_node);
  while (ros::ok()) { usleep(10000); }

  return 0;
}

#elif defined BUILDING_ROS2
namespace livox_ros
{
DriverNode::DriverNode(const rclcpp::NodeOptions & node_options)
: Node("livox_driver_node", node_options)
{
  DRIVER_INFO(*this, "Livox Ros Driver2 Version: %s", LIVOX_ROS_DRIVER2_VERSION_STRING);

  /** Init default system parameter */
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq = 10.0; /* Hz */
  int output_type = kOutputToRos;
  std::string frame_id;
  std::string lidar_qos_reliability = "reliable";
  std::string imu_qos_reliability = "reliable";
  int lidar_qos_depth = 256;
  int imu_qos_depth = 256;
  double imu_diagnostics_log_period_sec = 5.0;
  int imu_queue_warn_depth = 100;
  double imu_queue_warn_delay_ms = 100.0;
  double imu_timestamp_gap_warn_ms = 20.0;
  int pointcloud_poll_startup_delay_ms = 3000;
  int imu_poll_startup_delay_ms = 3000;

  this->declare_parameter("xfer_format", xfer_format);
  this->declare_parameter("multi_topic", 0);
  this->declare_parameter("data_src", data_src);
  this->declare_parameter("publish_freq", 10.0);
  this->declare_parameter("output_data_type", output_type);
  this->declare_parameter("frame_id", "frame_default");
  this->declare_parameter("user_config_path", "path_default");
  this->declare_parameter("cmdline_input_bd_code", "000000000000001");
  this->declare_parameter("lvx_file_path", "/home/livox/livox_test.lvx");
  this->declare_parameter(
      "lidar_qos_reliability", lidar_qos_reliability);
  this->declare_parameter("lidar_qos_depth", lidar_qos_depth);
  this->declare_parameter("imu_qos_reliability", imu_qos_reliability);
  this->declare_parameter("imu_qos_depth", imu_qos_depth);
  this->declare_parameter(
      "imu_diagnostics_log_period_sec", imu_diagnostics_log_period_sec);
  this->declare_parameter(
      "imu_queue_warn_depth", imu_queue_warn_depth);
  this->declare_parameter(
      "imu_queue_warn_delay_ms", imu_queue_warn_delay_ms);
  this->declare_parameter(
      "imu_timestamp_gap_warn_ms", imu_timestamp_gap_warn_ms);
  this->declare_parameter(
      "pointcloud_poll_startup_delay_ms",
      pointcloud_poll_startup_delay_ms);
  this->declare_parameter(
      "imu_poll_startup_delay_ms", imu_poll_startup_delay_ms);

  this->get_parameter("xfer_format", xfer_format);
  this->get_parameter("multi_topic", multi_topic);
  this->get_parameter("data_src", data_src);
  this->get_parameter("publish_freq", publish_freq);
  this->get_parameter("output_data_type", output_type);
  this->get_parameter("frame_id", frame_id);
  this->get_parameter(
      "lidar_qos_reliability", lidar_qos_reliability);
  this->get_parameter("lidar_qos_depth", lidar_qos_depth);
  this->get_parameter("imu_qos_reliability", imu_qos_reliability);
  this->get_parameter("imu_qos_depth", imu_qos_depth);
  this->get_parameter(
      "imu_diagnostics_log_period_sec", imu_diagnostics_log_period_sec);
  this->get_parameter("imu_queue_warn_depth", imu_queue_warn_depth);
  this->get_parameter(
      "imu_queue_warn_delay_ms", imu_queue_warn_delay_ms);
  this->get_parameter(
      "imu_timestamp_gap_warn_ms", imu_timestamp_gap_warn_ms);
  this->get_parameter(
      "pointcloud_poll_startup_delay_ms",
      pointcloud_poll_startup_delay_ms);
  this->get_parameter(
      "imu_poll_startup_delay_ms", imu_poll_startup_delay_ms);

  const auto valid_reliability = [](const std::string& value) {
    return value == "reliable" || value == "best_effort";
  };
  if (!valid_reliability(lidar_qos_reliability) ||
      !valid_reliability(imu_qos_reliability)) {
    throw std::invalid_argument(
        "Livox publisher reliability must be reliable or best_effort");
  }
  if (lidar_qos_depth <= 0 || imu_qos_depth <= 0 ||
      imu_diagnostics_log_period_sec <= 0.0 ||
      imu_queue_warn_depth <= 0 ||
      imu_queue_warn_delay_ms <= 0.0 ||
      imu_timestamp_gap_warn_ms <= 0.0 ||
      pointcloud_poll_startup_delay_ms < 0 ||
      imu_poll_startup_delay_ms < 0) {
    throw std::invalid_argument(
        "Livox QoS and diagnostics parameters must be positive");
  }
  pointcloud_poll_startup_delay_ms_ =
      static_cast<uint32_t>(pointcloud_poll_startup_delay_ms);
  imu_poll_startup_delay_ms_ =
      static_cast<uint32_t>(imu_poll_startup_delay_ms);

  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.5) {
    publish_freq = 0.5;
  } else {
    publish_freq = publish_freq;
  }

  future_ = exit_signal_.get_future();

  /** Lidar data distribute control and lidar data source set */
  lddc_ptr_ = std::make_unique<Lddc>(xfer_format, multi_topic, data_src, output_type, publish_freq, frame_id);
  lddc_ptr_->SetRosNode(this);

  if (data_src == kSourceRawLidar) {
    DRIVER_INFO(*this, "Data Source is raw lidar.");

    std::string user_config_path;
    this->get_parameter("user_config_path", user_config_path);
    DRIVER_INFO(*this, "Config file : %s", user_config_path.c_str());

    std::string cmdline_bd_code;
    this->get_parameter("cmdline_input_bd_code", cmdline_bd_code);

    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);
    lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));
    lddc_ptr_->ConfigureRos2(
        lidar_qos_reliability, static_cast<uint32_t>(lidar_qos_depth),
        imu_qos_reliability, static_cast<uint32_t>(imu_qos_depth),
        imu_diagnostics_log_period_sec,
        static_cast<uint32_t>(imu_queue_warn_depth),
        imu_queue_warn_delay_ms / 1000.0,
        static_cast<uint64_t>(imu_timestamp_gap_warn_ms * 1e6));
    DRIVER_INFO(
        *this,
        "Livox poll startup delays: pointcloud=%u ms imu=%u ms",
        pointcloud_poll_startup_delay_ms_, imu_poll_startup_delay_ms_);
    if ((read_lidar->InitLdsLidar(user_config_path))) {
      DRIVER_INFO(*this, "Init lds lidar success!");
    } else {
      DRIVER_ERROR(*this, "Init lds lidar fail!");
    }
    imudata_poll_thread_ =
        std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, this);
  } else {
    DRIVER_ERROR(*this, "Invalid data src (%d), please check the launch file", data_src);
  }

  pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, this);
}

}  // namespace livox_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(livox_ros::DriverNode)

#endif  // defined BUILDING_ROS2


void DriverNode::PointCloudDataPollThread()
{
  std::future_status status;
  if (pointcloud_poll_startup_delay_ms_ > 0) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(pointcloud_poll_startup_delay_ms_));
  }
  do {
    lddc_ptr_->DistributePointCloudData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}

void DriverNode::ImuDataPollThread()
{
  std::future_status status;
  if (imu_poll_startup_delay_ms_ > 0) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(imu_poll_startup_delay_ms_));
  }
  do {
    lddc_ptr_->DistributeImuData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}





















