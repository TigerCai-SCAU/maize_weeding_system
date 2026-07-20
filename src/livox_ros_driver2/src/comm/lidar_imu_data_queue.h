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

#ifndef LIVOX_ROS_DRIVER_LIDAR_IMU_DATA_QUEUE_H_
#define LIVOX_ROS_DRIVER_LIDAR_IMU_DATA_QUEUE_H_

#include <list>
#include <mutex>
#include <cstdint>
#include <cstddef>

namespace livox_ros {

// Based on the IMU Data Type in Livox communication protocol
// TODO: add a link to the protocol
typedef struct {
  float gyro_x;        /**< Gyroscope X axis, Unit:rad/s */
  float gyro_y;        /**< Gyroscope Y axis, Unit:rad/s */
  float gyro_z;        /**< Gyroscope Z axis, Unit:rad/s */
  float acc_x;         /**< Accelerometer X axis, Unit:g */
  float acc_y;         /**< Accelerometer Y axis, Unit:g */
  float acc_z;         /**< Accelerometer Z axis, Unit:g */
} RawImuPoint;

typedef struct {
  uint8_t lidar_type;
  uint32_t handle;
  uint8_t slot;
  // union {
  //   uint8_t handle;
  //   uint8_t slot;
  // };
  uint64_t time_stamp;
  float gyro_x;        /**< Gyroscope X axis, Unit:rad/s */
  float gyro_y;        /**< Gyroscope Y axis, Unit:rad/s */
  float gyro_z;        /**< Gyroscope Z axis, Unit:rad/s */
  float acc_x;         /**< Accelerometer X axis, Unit:g */
  float acc_y;         /**< Accelerometer Y axis, Unit:g */
  float acc_z;         /**< Accelerometer Z axis, Unit:g */
} ImuData;

struct ImuQueueDiagnostics {
  uint64_t pushed = 0;
  uint64_t popped = 0;
  uint64_t high_water_mark = 0;
  uint64_t timestamp_nonmonotonic = 0;
  uint64_t timestamp_large_gap = 0;
  uint64_t oldest_timestamp_ns = 0;
  size_t depth = 0;
};

class LidarImuDataQueue {
 public:
  void Push(ImuData* imu_data);
  bool Pop(ImuData& imu_data);
  bool Empty();
  size_t Clear();
  void SetTimestampGapThresholdNs(uint64_t threshold_ns);
  ImuQueueDiagnostics GetDiagnostics();

 private:
  std::mutex mutex_;
  std::list<ImuData> imu_data_queue_;
  uint64_t pushed_ = 0;
  uint64_t popped_ = 0;
  uint64_t high_water_mark_ = 0;
  uint64_t last_timestamp_ns_ = 0;
  uint64_t timestamp_gap_threshold_ns_ = 20'000'000;
  uint64_t timestamp_nonmonotonic_ = 0;
  uint64_t timestamp_large_gap_ = 0;
};

} // namespace

#endif // LIVOX_ROS_DRIVER_LIDAR_IMU_DATA_QUEUE_H_
