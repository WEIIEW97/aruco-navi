/*
 * Copyright (c) 2023--present, WILLIAM WEI.  All rights reserved.
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
#include "rscvcapture.h"
#include "calculate_angles.h"
#include "drawfunc.h"
// #include "kinematics.h"
// #include "mathtypes.h"

namespace aruconavi {
  //  using float3 = vec3<float>;

  bool rs_with_cv_capture(int win_width, int win_height) {
    try {
      //      if (!check_imu_is_supported()) {
      //        std::cerr << "Device supporting IMU not found" << std::endl;
      //        return false;
      //      }

      rs2::pipeline pipe;
      pipe.start();
      //      rs2::config cfg;
      //      cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
      //      cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
      //      RotationEstimator algo;
      //      auto profile = pipe.start(cfg, [&](const rs2::frame& frame) {
      //        // Cast the frame that arrived to motion frame
      //        auto motion = frame.as<rs2::motion_frame>();
      //        // If casting succeeded and the arrived frame is from gyro
      //        stream if (motion && motion.get_profile().stream_type() ==
      //        RS2_STREAM_GYRO &&
      //            motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
      //          // Get the timestamp of the current frame
      //          double ts = motion.get_timestamp();
      //          // Get gyro measures
      //          rs2_vector gyro_data = motion.get_motion_data();
      //          // Call function that computes the angle of motion based on
      //          the
      //          // retrieved measures
      //          algo.process_gyro(gyro_data, ts);
      //        }
      //        // If casting succeeded and the arrived frame is from
      //        accelerometer
      //        // stream
      //        if (motion && motion.get_profile().stream_type() ==
      //        RS2_STREAM_ACCEL &&
      //            motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
      //          // Get accelerometer measures
      //          rs2_vector accel_data = motion.get_motion_data();
      //          // Call function that computes the angle of motion based on
      //          the
      //          // retrieved measures
      //          algo.process_accel(accel_data);
      //        }
      //      });

      using namespace cv;
      const auto win_name = "display image";
      namedWindow(win_name, WINDOW_AUTOSIZE);
      aruconavi::Calc calculator(aruconavi::INTRINSICS640480,
                                 aruconavi::DISTORTION);
      Mat image_out;
      while (waitKey(1) < 0 &&
             getWindowProperty(win_name, WND_PROP_AUTOSIZE) >= 0) {
        rs2::frameset data = pipe.wait_for_frames(); // wait for next set of
                                                     // frames from the camera
        auto rgb = data.get_color_frame();

        // query frame size (width and height)
        const int w = rgb.as<rs2::video_frame>().get_width();
        const int h = rgb.as<rs2::video_frame>().get_height();

        Mat image(Size(w, h), CV_8UC3, (void*)rgb.get_data(), Mat::AUTO_STEP);
        calculator.process(image);
        cv::cvtColor(image, image_out, cv::COLOR_BGR2RGB);
        if (calculator.draw_ready_) {
          aruconavi::Painter painter(image_out);
          painter.draw_detected_markers(calculator.pairs_);
          painter.draw_coordinate_system(calculator.camera_matrix_,
                                         calculator.dist_coef_,
                                         calculator.rvec_, calculator.tvec_);
          painter.print_message(calculator.ypr, calculator.trans_cam2world);
        }
        imshow(win_name, image_out);
      }
      return true;
    } catch (const rs2::error& e) {
      std::cerr << "RealSense error calling " << e.get_failed_function() << "("
                << e.get_failed_args() << "):\n    " << e.what() << std::endl;
      return false;
    } catch (const std::exception& e) {
      std::cerr << e.what() << std::endl;
      return false;
    }
  }
} // namespace aruconavi