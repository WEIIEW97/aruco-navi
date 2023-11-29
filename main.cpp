#include "src/calculate_angles.h"
#include "src/rscapture.h"
#include "src/rscvcapture.h"
#include <filesystem>
#include <vector>
using namespace aruconavi;
namespace fs = std::filesystem;
using namespace std;

int main() {
  bool status = rs_with_cv_capture(1280, 720);
  if (status) {
    cout << "launch succeed!" << endl;
  } else {
    cout << "launch failed!" << endl;
  }
  return 0;
}
