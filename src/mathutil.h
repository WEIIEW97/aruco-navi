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

#ifndef BLIND_ASSIST_MATHUTIL_H
#define BLIND_ASSIST_MATHUTIL_H
#include <Eigen/Dense>
#include <cmath>

namespace aruconavi {
  Eigen::Vector3f R2ypr(const Eigen::Matrix3f& R);
  Eigen::Matrix3f ypr2R(const Eigen::Vector3f& ypr);
} // namespace aruconavi
#endif // BLIND_ASSIST_MATHUTIL_H
