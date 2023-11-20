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
#include <utility>
#include "detect.h"

namespace aruconavi {
  class Calc {
  public:
    Eigen::Vector3f ypr, trans_cam2world;
    Eigen::Matrix3f camera_matrix_;
    Eigen::Vector<float, 5> dist_coef_;
    std::vector<cv::Vec3d> rvecs_, tvecs_;
    cv::Vec3d rvec_, tvec_;
    bool draw_ready_ = false;
    std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>> pairs_;

  public:
    Calc(Intrinsic intrinsic,
         Eigen::Vector<float, 5> dist_coef) {
      camera_matrix_ = build_intrinsic(intrinsic);
      dist_coef_ = std::move(dist_coef);
    }

    void process(cv::Mat& image);

    ~Calc() = default;

  private:
    void
    get_rvecs_and_tvecs(const std::vector<std::vector<cv::Point2f>>& corners,
                        float marker_len);
    void get_ypr_and_translation(int id);
    void left_right_depature_warning(int id);
  };

} // namespace aruconavi
#endif // BLIND_ASSIST_CALCULATE_ANGLES_H
