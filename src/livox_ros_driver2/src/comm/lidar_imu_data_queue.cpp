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

#include "lidar_imu_data_queue.h"

namespace livox_ros {

void LidarImuDataQueue::Push(ImuData* imu_data) {
  ImuData data;
  data.lidar_type = imu_data->lidar_type;
  data.handle = imu_data->handle;
  data.time_stamp = imu_data->time_stamp;

  data.gyro_x = imu_data->gyro_x;
  data.gyro_y = imu_data->gyro_y;
  data.gyro_z = imu_data->gyro_z;

  data.acc_x = imu_data->acc_x;
  data.acc_y = imu_data->acc_y;
  data.acc_z = imu_data->acc_z;

  std::lock_guard<std::mutex> lock(mutex_);
  if (last_timestamp_ns_ != 0) {
    if (data.time_stamp <= last_timestamp_ns_) {
      ++timestamp_nonmonotonic_;
    } else if (data.time_stamp - last_timestamp_ns_ >
               timestamp_gap_threshold_ns_) {
      ++timestamp_large_gap_;
    }
  }
  last_timestamp_ns_ = data.time_stamp;
  imu_data_queue_.push_back(std::move(data));
  ++pushed_;
  if (imu_data_queue_.size() > high_water_mark_) {
    high_water_mark_ = imu_data_queue_.size();
  }
}

bool LidarImuDataQueue::Pop(ImuData& imu_data) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (imu_data_queue_.empty()) {
    return false;
  }
  imu_data = imu_data_queue_.front();
  imu_data_queue_.pop_front();
  ++popped_;
  return true;
}

bool LidarImuDataQueue::Empty() {
  std::lock_guard<std::mutex> lock(mutex_);
  return imu_data_queue_.empty();
}

size_t LidarImuDataQueue::Clear() {
  std::list<ImuData> tmp_imu_data_queue;
  size_t cleared = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cleared = imu_data_queue_.size();
    imu_data_queue_.swap(tmp_imu_data_queue);
  }
  return cleared;
}

void LidarImuDataQueue::SetTimestampGapThresholdNs(uint64_t threshold_ns) {
  std::lock_guard<std::mutex> lock(mutex_);
  timestamp_gap_threshold_ns_ = threshold_ns;
}

ImuQueueDiagnostics LidarImuDataQueue::GetDiagnostics() {
  std::lock_guard<std::mutex> lock(mutex_);
  ImuQueueDiagnostics diagnostics;
  diagnostics.pushed = pushed_;
  diagnostics.popped = popped_;
  diagnostics.high_water_mark = high_water_mark_;
  diagnostics.timestamp_nonmonotonic = timestamp_nonmonotonic_;
  diagnostics.timestamp_large_gap = timestamp_large_gap_;
  diagnostics.depth = imu_data_queue_.size();
  if (!imu_data_queue_.empty()) {
    diagnostics.oldest_timestamp_ns = imu_data_queue_.front().time_stamp;
  }
  return diagnostics;
}

} // namespace livox_ros
