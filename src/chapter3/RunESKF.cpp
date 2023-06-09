#include <glog/logging.h>

#include <iostream>

#include "ESKF.hpp"

int main(int argc, char const *argv[])
{
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  ESKFD::Options option;
  ESKFD eskf;
  Sophus::SO3d so3;
  std::cout << so3.unit_quaternion().coeffs().transpose() << std::endl;
  Eigen::Vector3d init_bg, init_ba, g;
  eskf.SetInitialConditions(option, init_bg, init_ba, g);
  return 0;
}
