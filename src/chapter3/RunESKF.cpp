#include "ESKF.hpp"
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/se3.hpp>

int main(int argc, char const *argv[])
{
  std::cout << "Hello world\n";
  std::cout << WEIHAO_SOPHUS;
  Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  return 0;
}
