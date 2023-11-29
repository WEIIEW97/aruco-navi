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

#ifndef ARUCO_NAVI_MATHTYPES_H
#define ARUCO_NAVI_MATHTYPES_H

#include <iostream>

namespace aruconavi {
  template <typename T>
  class vec3 {
  public:
    T e[3];
    vec3() : e{0, 0, 0} {}
    vec3(T e0, T e1, T e2) : e{e0, e1, e2} {}

    T x() const { return e[0]; }
    T y() const { return e[1]; }
    T z() const { return e[2]; }

    vec3 operator-() const { return vec3(-e[0], -e[1], -e[2]); }
    T operator[](int i) const { return e[i]; }
    T& operator[](int i) { return e[i]; }

    vec3& operator+=(const vec3& v) {
      e[0] += v.e[0];
      e[1] += v.e[1];
      e[2] += v.e[2];
      return *this;
    }

    vec3& operator*=(T t) {
      e[0] *= t;
      e[1] *= t;
      e[2] *= t;
      return *this;
    }

    vec3& operator/=(T t) { return *this *= 1 / t; }

    T length() const { return sqrt(length_squared()); }

    T length_squared() const { return e[0] * e[0] + e[1] * e[1] + e[2] * e[2]; }

    bool near_zero() const {
      // Return true if the vector is close to zero in all dimensions.
      auto s = 1e-8;
      return (fabs(e[0]) < s) && (fabs(e[1]) < s) && (fabs(e[2]) < s);
    }

    void add(T t1, T t2, T t3) {
      e[0] += t1;
      e[1] += t2;
      e[2] += t3;
    }
  };

  template <typename T>
  inline std::ostream& operator<<(std::ostream& out, const vec3<T>& v) {
    return out << v.e[0] << ' ' << v.e[1] << ' ' << v.e[2];
  }

  template <typename T>
  inline vec3<T> operator+(const vec3<T>& u, const vec3<T>& v) {
    return vec3(u.e[0] + v.e[0], u.e[1] + v.e[1], u.e[2] + v.e[2]);
  }

  template <typename T>
  inline vec3<T> operator-(const vec3<T>& u, const vec3<T>& v) {
    return vec3(u.e[0] - v.e[0], u.e[1] - v.e[1], u.e[2] - v.e[2]);
  }

  template <typename T>
  inline vec3<T> operator*(const vec3<T>& u, const vec3<T>& v) {
    return vec3(u.e[0] * v.e[0], u.e[1] * v.e[1], u.e[2] * v.e[2]);
  }

  template <typename T>
  inline vec3<T> operator*(T t, const vec3<T>& v) {
    return vec3(t * v.e[0], t * v.e[1], t * v.e[2]);
  }

  template <typename T>
  inline vec3<T> operator*(const vec3<T>& v, T t) {
    return t * v;
  }

  template <typename T>
  inline vec3<T> operator/(vec3<T> v, T t) {
    return (1 / t) * v;
  }

  template <typename T>
  inline T dot(const vec3<T>& u, const vec3<T>& v) {
    return u.e[0] * v.e[0] + u.e[1] * v.e[1] + u.e[2] * v.e[2];
  }

  template <typename T>
  inline vec3<T> cross(const vec3<T>& u, const vec3<T>& v) {
    return vec3(u.e[1] * v.e[2] - u.e[2] * v.e[1],
                u.e[2] * v.e[0] - u.e[0] * v.e[2],
                u.e[0] * v.e[1] - u.e[1] * v.e[0]);
  }

  template <typename T>
  inline vec3<T> unit_vector(vec3<T> v) {
    return v / v.length();
  }
} // namespace aruconavi

#endif // ARUCO_NAVI_MATHTYPES_H
