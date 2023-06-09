#ifndef _DATA_TYPE_HPP__
#define _DATA_TYPE_HPP__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>

struct IMU
{
  IMU() = default;
  IMU(double t, const Eigen::Vector3d& gyro, const Eigen::Vector3d& acce)
      : timestamp_(t), gyro_(gyro), acce_(acce)
  {
  }
  double timestamp_ = 0.0;
  Eigen::Vector3d gyro_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d acce_ = Eigen::Vector3d::Zero();
};

#endif  //_DATA_TYPE_HPP__