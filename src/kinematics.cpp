/*
 * Copyright (c) 2022-2023, William Wei. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "kinematics.h"
#include <iostream>

namespace aruconavi {
  using float3 = vec3<float>;

  bool check_imu_is_supported() {
    bool found_gyro = false;
    bool found_accel = false;
    rs2::context ctx;
    for (auto dev : ctx.query_devices()) {
      // The same device should support gyro and accel
      found_gyro = false;
      found_accel = false;
      for (auto sensor : dev.query_sensors()) {
        for (auto profile : sensor.get_stream_profiles()) {
          if (profile.stream_type() == RS2_STREAM_GYRO)
            found_gyro = true;

          if (profile.stream_type() == RS2_STREAM_ACCEL)
            found_accel = true;
        }
      }
      if (found_gyro && found_accel)
        break;
    }
    return found_gyro && found_accel;
  }

  void RotationEstimator::process_gyro(rs2_vector gyro_data, double ts) {
    if (first_gyro) {
      first_gyro = false;
      last_ts_gyro = ts;
    }
    // Holds the change in angle, as calculated from gyro
    float3 gyro_angle;

    gyro_angle.e[0] = gyro_data.x; // pitch
    gyro_angle.e[1] = gyro_data.y; // yaw
    gyro_angle.e[2] = gyro_data.z; // roll

    // Compute the difference between arrival times of previous and current gyro
    // frames
    double dt_gyro = (ts - last_ts_gyro) / 1000.0;
    last_ts_gyro = ts;

    // Change in angle equals gyro measures * time passed since last measurement
    gyro_angle = gyro_angle * static_cast<float>(dt_gyro);
    // Apply the calculated change of angle to the current angle (theta)
    std::lock_guard<std::mutex> lock(theta_mutex);
    theta.add(-gyro_angle.z(), -gyro_angle.y(), -gyro_angle.x());
  }

  void RotationEstimator::process_accel(rs2_vector accel_data) {
    float3 accel_angle;
    // Calculate rotation angle from accelerometer data
    accel_angle.e[2] = atan2(accel_data.y, accel_data.z);
    accel_angle.e[0] = atan2(accel_data.x, sqrt(accel_data.y * accel_data.y +
                                                accel_data.z * accel_data.z));
    // If it is the first iteration, set initial pose of camera according to
    // accelerometer data (note the different handling for Y axis)
    std::lock_guard<std::mutex> lock(theta_mutex);
    if (first_accel) {
      first_accel = false;
      theta = accel_angle;
      // Since we can't infer the angle around Y axis using accelerometer data,
      // we'll use PI as a convetion for the initial pose
      theta.e[1] = M_PI;
    } else {
      /*
      Apply Complementary Filter:
          - high-pass filter = theta * alpha:  allows short-duration signals to
      pass through while filtering out signals that are steady over time, is
      used to cancel out drift.
          - low-pass filter = accel * (1- alpha): lets through long term
      changes, filtering out short term fluctuations
      */
      theta.e[0] = theta.e[0] * alpha + accel_angle.e[0] * (1 - alpha);
      theta.e[2] = theta.e[2] * alpha + accel_angle.e[2] * (1 - alpha);
    }
  }

  float3 RotationEstimator::get_theta() {
    std::lock_guard<std::mutex> lock(theta_mutex);
    return theta;
  }

  void RotationEstimator::print_theta() {
    float3 cur_rot = get_theta();
    std::cout << "realsense device rotate in yall: " << cur_rot.y() << "\n"
    << "realsense device rotate in pitch: " << cur_rot.x() << "\n"
    << "realsense device rotate in roll: " << cur_rot.z() << std::endl;
  }
} // namespace aruconavi
