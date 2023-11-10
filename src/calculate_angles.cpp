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

void get_rvecs_and_tvecs(std::vector<cv::Vec3d>& rvecs,
                         std::vector<cv::Vec3d>& tvecs,
                         const std::vector<std::vector<cv::Point2f>>& corners,
                         float marker_len,
                         const Eigen::Matrix3f& intrinsic_matrix,
                         const Eigen::Vector<float, 5>& dist_coef) {
  cv::Mat intrinsic_matrix_cv, dist_coef_cv;
  cv::eigen2cv(intrinsic_matrix, intrinsic_matrix_cv);
  cv::eigen2cv(dist_coef, dist_coef_cv);
  cv::aruco::estimatePoseSingleMarkers(corners, marker_len, intrinsic_matrix_cv,
                                       dist_coef_cv, rvecs, tvecs);
}

void get_ypr_and_translation(Eigen::Vector3f& ypr,
                             Eigen::Vector3f& trans_cam2world,
                             const std::vector<cv::Vec3d>& rvecs,
                             const std::vector<cv::Vec3d>& tvecs, int id) {
  //  cv::Mat rvecs, tvecs;

  cv::Mat rotvec;
  // convert rvec to rotation matrix
  cv::Rodrigues(rvecs[id], rotvec);
  // inverse the world2cam to cam2world
  Eigen::Matrix3d rotvec_eigen, rot_cam2world;
  Eigen::Vector3d tvec_eigen;
  cv::cv2eigen(rotvec, rotvec_eigen);
  cv::cv2eigen(tvecs[id], tvec_eigen);
  rot_cam2world = rotvec_eigen.transpose();
  Eigen::Matrix3f rot_cam2world_ = rot_cam2world.cast<float>();
  Eigen::Vector3f tvec_eigen_ = tvec_eigen.cast<float>();
  trans_cam2world = -rot_cam2world_ * tvec_eigen_;
  ypr = R2ypr(rot_cam2world_);
}

void draw_coordinate_system(cv::Mat& image, const Eigen::Matrix3f& intrinsic_matrix, const Eigen::Vector<float, 5>& dist_coef, cv::Vec3d rvec, cv::Vec3d tvec) {
  cv::Mat intrinsic_matrix_cv, dist_coef_cv;
  cv::eigen2cv(intrinsic_matrix, intrinsic_matrix_cv);
  cv::eigen2cv(dist_coef, dist_coef_cv);
  cv::drawFrameAxes(image, intrinsic_matrix_cv, dist_coef_cv, rvec, tvec, 0.1);
}

void process(const std::string& img_path,
             IntelRealsenseIntrinsics640_480& intrinsic,
             const Eigen::Vector<float, 5>& dist,
             bool is_visualize) {
  cv::Mat image = cv::imread(img_path, cv::IMREAD_ANYCOLOR);
  std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>> pairs;
  pairs = detect_markers(image, cv::aruco::DICT_6X6_250);

  int chosen_id = find_min_id(pairs);

  Eigen::Matrix3f camera_matrix;
  camera_matrix = build_intrinsic(intrinsic);

  Eigen::Vector3f ypr, trans_cam2world;
  float marker_len = 0.176;

  std::vector<cv::Vec3d> rvecs, tvecs;
  get_rvecs_and_tvecs(rvecs, tvecs, pairs.second, marker_len, camera_matrix,
                      distortion);
  get_ypr_and_translation(ypr, trans_cam2world, rvecs, tvecs, chosen_id);
  std::cout << "yaw, pitch, roll is: \n" << ypr << std::endl;
  if (is_visualize) {
    cv::Mat imagec;
    image.copyTo(imagec);
    draw_coordinate_system(imagec, camera_matrix, distortion, rvecs[chosen_id], tvecs[chosen_id]);
    cv::imshow("axis figure", imagec);
    cv::waitKey(0);
    cv::destroyAllWindows();
  }
}