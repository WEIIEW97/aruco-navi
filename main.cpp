#include "src/calculate_angles.h"
#include "src/rscapture.h"
#include "src/rscvcapture.h"
#include <filesystem>
#include <vector>
using namespace aruconavi;
namespace fs = std::filesystem;
using namespace std;

#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN    "\033[36m"
#define WHITE   "\033[37m"

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
//  string img_path = "/home/william/Codes/aruco-navi/aruco_capture/frame_0139.png";
//  Calc calculator(INTRINSICS640480, DISTORTION);
//  cv::Mat image = cv::imread(img_path, cv::IMREAD_ANYCOLOR);
//  calculator.process(image);
//  cout << calculator.ypr << endl;
//  cout << calculator.trans_cam2world << endl;
//
//  cv::Mat image_copy;
//  image.copyTo(image_copy);
//  draw_coordinate_system(image_copy, calculator.camera_matrix_, calculator.dist_coef_, calculator.rvec_, calculator.tvec_);
//  cv::imshow("detected coordinate system", image_copy);
//  cv::waitKey(0);
//  cv::destroyAllWindows();
//  bool status = rs_with_cv_capture(1280, 720);
//  bool status = rs_capture(1280, 720);
//  if (status) {
//    cout << "launch succeed!" << endl;
//  } else {
//    cout << "launch failed!" << endl;
//  }
  cv::Point2f p1, p2, p3, p4;
  p1 = {242, 434};
  p2 = {0, 336};
  p3 = {166, 172};
  p4 = {335, 430};

  std::vector<cv::Point2f> corner;
  corner.push_back(p1);
  corner.push_back(p2);
  corner.push_back(p3);
  corner.push_back(p4);

  float x=0, y=0;
  for (const auto& point : corner) {
    x+=point.x;
    y+=point.y;
  }
  if (!corner.empty()) {
    x /= corner.size();
    y /= corner.size();
  }
  cout << "x: " << x << " y: " << y << endl;
  return 0;
}
