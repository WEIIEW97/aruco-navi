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

#ifndef BLIND_ASSIST_CALCULATE_ANGLES_H
#define BLIND_ASSIST_CALCULATE_ANGLES_H
#include "detect.h"

void get_rvecs_and_tvecs(std::vector<cv::Vec3d>& rvecs,
                         std::vector<cv::Vec3d>& tvecs,
                         const std::vector<std::vector<cv::Point2f>>& corners,
                         float marker_len,
                         const Eigen::Matrix3f& intrinsic_matrix,
                         const Eigen::Vector<float, 5>& dist_coef);
void get_ypr_and_translation(Eigen::Vector3f& ypr,
                             Eigen::Vector3f& trans_cam2world,
                             const std::vector<cv::Vec3d>& rvecs,
                             const std::vector<cv::Vec3d>& tvecs, int id);
void process(const std::string& img_path,
             IntelRealsenseIntrinsics640_480& intrinsic,
             const Eigen::Vector<float, 5>& dist,
             bool is_visualize= false);

#endif // BLIND_ASSIST_CALCULATE_ANGLES_H
