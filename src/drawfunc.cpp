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
#include "drawfunc.h"
#include <opencv2/core/eigen.hpp>
#include <opencv2/aruco.hpp>
#include <vector>

namespace aruconavi {
  void Painter::draw_coordinate_system(const Eigen::Matrix3f& intrinsic_matrix,
                                       const Eigen::Vector<float, 5>& dist_coef,
                                       const cv::Vec3d& rvec,
                                       const cv::Vec3d& tvec) {
    cv::Mat intrinsic_matrix_cv, dist_coef_cv;
    cv::eigen2cv(intrinsic_matrix, intrinsic_matrix_cv);
    cv::eigen2cv(dist_coef, dist_coef_cv);
    cv::drawFrameAxes(image_, intrinsic_matrix_cv, dist_coef_cv, rvec, tvec,
                      0.1);
  }

  void Painter::draw_detected_markers(
      const std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>>&
          pairs) {
    std::vector<int> marked_ids = pairs.first;
    std::vector<std::vector<cv::Point2f>> marked_corners = pairs.second;
    if (!marked_ids.empty()) {
      cv::aruco::drawDetectedMarkers(image_, marked_corners, marked_ids);
    }
  }

  void Painter::print_message(const Eigen::Vector3f& ypr, const Eigen::Vector3f& translation) {
    std::cout << "rotation angles are: " << "\n" <<
        "yaw: " << ypr(0) << "\n" <<
        "pitch: " << ypr(1) << "\n" <<
        "roll: " << ypr(2) << std::endl;
    std::cout << "translation offset is(in meters): " << "\n" <<
        "x axis: " << translation(0) << "\n" <<
        "y axis: " << translation(1) << "\n" <<
        "z axis: " << translation(2) << std::endl;
    std::cout << "===============================" << std::endl;
  }

} // namespace aruconavi