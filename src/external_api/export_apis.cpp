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
#include "export_apis.h"
#include <opencv2/core/eigen.hpp>

#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN    "\033[36m"
#define WHITE   "\033[37m"

namespace py = pybind11;

std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>>
#if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 8
detect_markers(cv::Mat& image,
               cv::aruco::PredefinedDictionaryType aruco_dict_info) {
#else
detect_markers(cv::Mat& image,
               cv::aruco::PREDEFINED_DICTIONARY_NAME aruco_dict_info) {
#endif
  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
#if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 8
  auto aruco_info = cv::aruco::getPredefinedDictionary(aruco_dict_info);
  cv::Ptr<cv::aruco::Dictionary> aruco_info_ptr =
      cv::makePtr<cv::aruco::Dictionary>(aruco_info);
#else
  auto aruco_info = cv::aruco::getPredefinedDictionary(aruco_dict_info);
#endif
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;

#if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 8
  cv::aruco::detectMarkers(gray, aruco_info_ptr, corners, ids);
#else
  cv::aruco::detectMarkers(gray, aruco_info, corners, ids);
#endif
  return std::make_pair(ids, corners);
}

int find_min_id(const std::pair<std::vector<int>,
                                std::vector<std::vector<cv::Point2f>>>& pairs) {
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

int detect_left_right_departure(int id) {
  // if id % 10 == 3 which means it turns into left
  // where id % 10 == 4 means it turns into right
  if (id % 10 == 3) {
    return 3;
  } else if (id % 10 == 4) {
    return 4;
  } else {
    return 0;
  }
}

Eigen::Matrix3f build_intrinsic(Intrinsic& params) {
  Eigen::Matrix3f intrinsic;
  intrinsic << params.fx, 0.f, params.cx, 0.f, params.fy, params.cy, 0.f, 0.f,
      1;
  return intrinsic;
}

std::vector<cv::Point2f> aruco_center_localization(
    const std::vector<std::vector<cv::Point2f>>& corners) {
  std::vector<cv::Point2f> center_points(corners.size());

  for (const auto& corner : corners) {
    float x = 0, y = 0;
    for (const auto& point : corner) {
      x += point.x;
      y += point.y;
    }
    auto n = corner.size();
    x /= n;
    y /= n;
    center_points.emplace_back(x, y);
  }
  return center_points;
}

Eigen::Vector3f R2ypr(const Eigen::Matrix3f& R) {
  Eigen::Vector3f n = R.col(0);
  Eigen::Vector3f o = R.col(1);
  Eigen::Vector3f a = R.col(2);

  Eigen::Vector3f ypr(3);
  double y = atan2(n(1), n(0));
  double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
  double r =
      atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
  ypr(0) = y;
  ypr(1) = p;
  ypr(2) = r;

  return ypr / M_PI * 180.0;
}

Eigen::Matrix3f ypr2R(const Eigen::Vector3f& ypr) {
  auto y = ypr(0) / 180.0 * M_PI;
  auto p = ypr(1) / 180.0 * M_PI;
  auto r = ypr(2) / 180.0 * M_PI;

  Eigen::Matrix3f Rz, Ry, Rx;
  Rz << cos(y), -sin(y), 0, sin(y), cos(y), 0, 0, 0, 1;
  Ry << cos(p), 0., sin(p), 0., 1., 0., -sin(p), 0., cos(p);
  Rx << 1., 0., 0., 0., cos(r), -sin(r), 0., sin(r), cos(r);
  return Rz * Ry * Rx;
}

void PyCalc::get_rvecs_and_tvecs(
    const std::vector<std::vector<cv::Point2f>>& corners) {
  cv::Mat intrinsic_matrix_cv, dist_coef_cv;
  cv::eigen2cv(camera_matrix_, intrinsic_matrix_cv);
  cv::eigen2cv(dist_coef_, dist_coef_cv);
  cv::aruco::estimatePoseSingleMarkers(
      corners, marker_len_, intrinsic_matrix_cv, dist_coef_cv, rvecs_, tvecs_);
}

void PyCalc::get_ypr_and_translation(int id) {
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

std::tuple<Eigen::Vector3f, Eigen::Vector3f>
PyCalc::process(py::array_t<uint8_t>& image) {
  std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>> pairs;
  auto rows = image.shape(0);
  auto cols = image.shape(1);
  auto type = CV_8UC3;
  cv::Mat cvimage(rows, cols, type, (uint8_t*)image.data());
  pairs = detect_markers(cvimage, ArucoDict);

  pairs_ = pairs;
  int chosen_id = find_min_id(pairs);
  if (chosen_id >= 0) {
    this->get_rvecs_and_tvecs(pairs.second);
    this->get_ypr_and_translation(chosen_id);
    this->left_right_depature_warning(chosen_id);
    draw_ready_ = true;
  } else {
    trans_cam2world = {0, 0, 0};
    ypr = {0, 0, 0};
    draw_ready_ = false;
  }
  return std::make_tuple(ypr, trans_cam2world);
}

void PyCalc::left_right_depature_warning(int id) {
  int status = -1;
  status = detect_left_right_departure(id);
  if (status == 3) {
    std::cout << RED << "WARNING: "
              << "you are left drifted, please make a right turn!" << std::endl;
  }
  if (status == 4) {
    std::cout << RED << "WARNING: "
              << "you are right drifted, please make a left turn!" << std::endl;
  }
}

PYBIND11_MODULE(pyaruconavi, m) {
  py::class_<PyCalc>(m, "PyCalc")
      .def(py::init<Eigen::Vector<float, 4>, Eigen::Vector<float, 5>, float>())
      .def("process", &PyCalc::process);
}