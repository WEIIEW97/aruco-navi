#include "src/calculate_angles.h"
#include <filesystem>
#include <vector>
using namespace aruconavi;
namespace fs = std::filesystem;
using namespace std;
int main() {
//  string img_dir = "/Users/williamwei/Codes/aruco-navi/aruco_capture";
//  vector<string> img_names;
//  if (fs::is_directory(img_dir)) {
//    for(const auto& entry : fs::directory_iterator(img_dir)) {
//      img_names.emplace_back(entry.path().filename());
//    }
//  }
//
//  for(const auto& name : img_names) {
//    string full_nanme = img_dir + "/" + name;
//
//  }
  string img_path =
      "/Users/williamwei/Codes/aruco-navi/aruco_capture/frame_0139.png";
  Calc calculator(intrinsics640480, distortion);
  cv::Mat image = cv::imread(img_path, cv::IMREAD_ANYCOLOR);
  calculator.process(image);
  cout << calculator.ypr << endl;
  cout << calculator.trans_cam2world << endl;

  cv::Mat image_copy;
  image.copyTo(image_copy);
  draw_coordinate_system(image_copy, calculator.camera_matrix_, calculator.dist_coef_, calculator.rvec_, calculator.tvec_);
  cv::imshow("detected coordinate system", image_copy);
  cv::waitKey(0);
  cv::destroyAllWindows();
  return 0;
}
