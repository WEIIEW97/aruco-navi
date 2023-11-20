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

#ifndef ARUCO_NAVI_EXPORT_APIS_H
#define ARUCO_NAVI_EXPORT_APIS_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
// #include "../detect.h"
// #include "../mathutil.h"
// #include "../constant.h"

struct Intrinsic {
  float fx = 610.117;
  float fy = 608.71;
  float cx = 316.156;
  float cy = 249.345;
};

#if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 8
const cv::aruco::PredefinedDictionaryType ArucoDict = cv::aruco::DICT_6X6_250;
#else
const cv::aruco::PREDEFINED_DICTIONARY_NAME ArucoDict = cv::aruco::DICT_6X6_250;
#endif

std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>>
#if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 8
detect_markers(cv::Mat& image,
               cv::aruco::PredefinedDictionaryType aruco_dict_info);
#else
detect_markers(cv::Mat& image,
               cv::aruco::PREDEFINED_DICTIONARY_NAME aruco_dict_info);
#endif

Eigen::Vector3f R2ypr(const Eigen::Matrix3f& R);
Eigen::Matrix3f ypr2R(const Eigen::Vector3f& ypr);

int find_min_id(const std::pair<std::vector<int>,
                                std::vector<std::vector<cv::Point2f>>>& pairs);
int detect_left_right_departure(int id);
Eigen::Matrix3f build_intrinsic(Intrinsic& params);
std::vector<cv::Point2f>
aruco_center_localization(const std::vector<std::vector<cv::Point2f>>& corners);

class PyCalc {
public:
  Eigen::Vector3f ypr, trans_cam2world;
  Eigen::Matrix3f camera_matrix_;
  Eigen::Vector<float, 5> dist_coef_;
  std::vector<cv::Vec3d> rvecs_, tvecs_;
  cv::Vec3d rvec_, tvec_;
  bool draw_ready_ = false;
  std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>> pairs_;
  float marker_len_;

public:
  PyCalc(Eigen::Vector<float, 4> intrinsic, Eigen::Vector<float, 5> dist_coef,
         float marker_len) {
    Intrinsic intrinsic_;
    intrinsic_.fx = intrinsic(0);
    intrinsic_.fy = intrinsic(1);
    intrinsic_.cx = intrinsic(2);
    intrinsic_.cy = intrinsic(3);
    camera_matrix_ = build_intrinsic(intrinsic_);
    dist_coef_ = std::move(dist_coef);
    marker_len_ = marker_len;
  }

  std::tuple<Eigen::Vector3f, Eigen::Vector3f>
  process(pybind11::array_t<uint8_t>& image);

  ~PyCalc() = default;

private:
  void
  get_rvecs_and_tvecs(const std::vector<std::vector<cv::Point2f>>& corners);
  void get_ypr_and_translation(int id);
  void left_right_depature_warning(int id);
};

#endif // ARUCO_NAVI_EXPORT_APIS_H
