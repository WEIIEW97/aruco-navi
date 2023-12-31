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

#ifndef BLIND_ASSIST_DETECT_H
#define BLIND_ASSIST_DETECT_H

#include <utility>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/version.hpp>
#include "mathutil.h"
#include "constant.h"

namespace aruconavi {
  std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>>
#if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 8
  detect_markers(cv::Mat& image,
                 cv::aruco::PredefinedDictionaryType aruco_dict_info);
#else
  detect_markers(cv::Mat& image,
                 cv::aruco::PREDEFINED_DICTIONARY_NAME aruco_dict_info);
#endif

  std::vector<cv::Point2f> find_min_id_corners(
      const std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>>&
          pairs);
  int find_min_id(
      const std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>>&
          pairs);
  int detect_left_right_departure(int id);
  Eigen::Matrix3f build_intrinsic(Intrinsic& params);
  std::vector<cv::Point2f> aruco_center_localization(
      const std::vector<std::vector<cv::Point2f>>& corners);
} // namespace aruconavi
#endif // BLIND_ASSIST_DETECT_H
