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

#ifndef BLIND_ASSIST_CONSTANT_H
#define BLIND_ASSIST_CONSTANT_H
#include <Eigen/Dense>

struct IntelRealsenseIntrinsics640_480 {
  float fx = 610.117;
  float fy = 608.71;
  float cx = 316.156;
  float cy = 249.345;
};

static IntelRealsenseIntrinsics640_480 intrinsics640480;

static Eigen::Vector<float, 5> distortion = {0, 0, 0, 0, 0};

const cv::aruco::PREDEFINED_DICTIONARY_NAME ArucoDict = cv::aruco::DICT_6X6_250;
#endif // BLIND_ASSIST_CONSTANT_H
