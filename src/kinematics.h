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

#ifndef ARUCO_NAVI_KINEMATICS_H
#define ARUCO_NAVI_KINEMATICS_H
#include <librealsense2/rs.hpp>
#include <mutex>
#include "mathtypes.h"

namespace aruconavi {

  class RotationEstimator {
    using float3 = vec3<float>;
    // theta is the angle of camera rotation in x, y and z components
    float3 theta;
    std::mutex theta_mutex;
    /* alpha indicates the part that gyro and accelerometer take in computation
    of theta; higher alpha gives more weight to gyro, but too high values cause
    drift; lower alpha gives more weight to accelerometer, which is more
    sensitive to disturbances */
    float alpha = 0.98f;
    bool first_gyro = true;
    bool first_accel = true;
    // keeps the arrival time of previous gyro frame
    double last_ts_gyro = 0;

  public:
    void process_gyro(rs2_vector gyro_data, double ts);
    void process_accel(rs2_vector accel_data);
    float3 get_theta();
    void print_theta();
  };

  bool check_imu_is_supported();
} // namespace aruconavi
#endif // ARUCO_NAVI_KINEMATICS_H
