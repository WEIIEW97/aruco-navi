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

#ifndef BLIND_ASSIST_DRAWFUNC_H
#define BLIND_ASSIST_DRAWFUNC_H
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

namespace aruconavi {
  void draw_coordinate_system(cv::Mat& image,
                              const Eigen::Matrix3f& intrinsic_matrix,
                              const Eigen::Vector<float, 5>& dist_coef,
                              const cv::Vec3d& rvec, const cv::Vec3d& tvec);

  class Painter {
  public:
    cv::Mat image_;

  public:
    Painter(cv::Mat image) : image_(image) {}
    // for coordinate axis, we only draw the nearest one(i.e. the smallest id)
    void draw_coordinate_system(const Eigen::Matrix3f& intrinsic_matrix,
                                const Eigen::Vector<float, 5>& dist_coef,
                                const cv::Vec3d& rvec, const cv::Vec3d& tvec);
    void draw_detected_markers(
        const std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>>&
            pairs);
    void print_message(const Eigen::Vector3f& ypr, const Eigen::Vector3f& translation);
    //    void draw_text();
    ~Painter() = default;
  };
} // namespace aruconavi
#endif // BLIND_ASSIST_DRAWFUNC_H
