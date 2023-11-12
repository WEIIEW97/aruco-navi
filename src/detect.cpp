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
#include "detect.h"
#include <climits>

namespace aruconavi {
  std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>>
  detect_markers(cv::Mat& image,
                 cv::aruco::PREDEFINED_DICTIONARY_NAME aruco_dict_info) {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    auto aruco_info = cv::aruco::getPredefinedDictionary(aruco_dict_info);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    cv::aruco::detectMarkers(gray, aruco_info, corners, ids);
    return std::make_pair(ids, corners);
  }

  std::vector<cv::Point2f> find_min_id_corners(
      const std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>>&
          pairs) {
    int min_id = INT_MAX;
    int min_index = -1;

    const auto& ids = pairs.first;
    const auto& corners = pairs.second;

    for (int i = 0; i < ids.size(); ++i) {
      if (ids[i] < min_id) {
        min_id = ids[i];
        min_index = i;
      }
    }

    if (min_index == -1) {
      return {}; // Return empty if no IDs are found
    }

    return corners[min_index];
  }

  int find_min_id(
      const std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>>&
          pairs) {
    int min_id = INT_MAX;
    int min_index = -1;

    const auto& ids = pairs.first;
    const auto& corners = pairs.second;

    for (int i = 0; i < ids.size(); ++i) {
      if (ids[i] < min_id) {
        min_id = ids[i];
        min_index = i;
      }
    }

    return min_index;
  }

  Eigen::Matrix3f build_intrinsic(IntelRealsenseIntrinsics640_480& params) {
    Eigen::Matrix3f intrinsic;
    intrinsic << params.fx, 0.f, params.cx, 0.f, params.fy, params.cy, 0.f, 0.f,
        1;
    return intrinsic;
  }
} // namespace aruconavi
