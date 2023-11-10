#include "src/calculate_angles.h"

int main() {
    std::string img_path = "/home/william/Codes/LDW/data/aruco_capture/frame_0194.png";
    process(img_path, intrinsics640480, distortion, true);
    return 0;
}
