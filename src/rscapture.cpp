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
#include "rscapture.h"
#include "realsenssutil.h"

bool realsense_capture(int window_width, int window_height) {
  try {
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    window app(1280, 720, "realsense capture example");

    rs2::colorizer color_map;
    rs2::rates_printer printer;

    rs2::pipeline pipe;

    pipe.start();

    while (app) {
      rs2::frameset data =
          pipe.wait_for_frames().apply_filter(printer).apply_filter(color_map);
      app.show(data);
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