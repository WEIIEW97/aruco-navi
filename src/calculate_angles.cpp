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
#include "calculate_angles.h"
#include <opencv2/core/eigen.hpp>
#include <iostream>

namespace aruconavi {
  void Calc::get_rvecs_and_tvecs(
      const std::vector<std::vector<cv::Point2f>>& corners, float marker_len) {
    cv::Mat intrinsic_matrix_cv, dist_coef_cv;
    cv::eigen2cv(camera_matrix_, intrinsic_matrix_cv);
    cv::eigen2cv(dist_coef_, dist_coef_cv);
    cv::aruco::estimatePoseSingleMarkers(
        corners, marker_len, intrinsic_matrix_cv, dist_coef_cv, rvecs_, tvecs_);
  }

  void Calc::get_ypr_and_translation(int id) {
    cv::Mat rotvec;
    rvec_ = rvecs_[id];
    tvec_ = tvecs_[id];
    // convert rvec to rotation matrix
    cv::Rodrigues(rvec_, rotvec);
    // inverse the world2cam to cam2world
    Eigen::Matrix3d rotvec_eigen, rot_cam2world;
    Eigen::Vector3d tvec_eigen;
    cv::cv2eigen(rotvec, rotvec_eigen);
    cv::cv2eigen(tvec_, tvec_eigen);
    rot_cam2world = rotvec_eigen.transpose();
    Eigen::Matrix3f rot_cam2world_ = rot_cam2world.cast<float>();
    Eigen::Vector3f tvec_eigen_ = tvec_eigen.cast<float>();
    trans_cam2world = -rot_cam2world_ * tvec_eigen_;
    ypr = R2ypr(rot_cam2world_);
  }

  void Calc::process(cv::Mat& image) {
    std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>> pairs;
    pairs = detect_markers(image, ArucoDict);
    pairs_ = pairs;
    int chosen_id = find_min_id(pairs);
    if (chosen_id >= 0) {
      this->get_rvecs_and_tvecs(pairs.second, MarkerLen);
      this->get_ypr_and_translation(chosen_id);
      draw_ready_ = true;
    } else {
      trans_cam2world = {0, 0, 0};
      ypr = {0, 0, 0};
      draw_ready_ = false;
    }
  }
} // namespace aruconavi