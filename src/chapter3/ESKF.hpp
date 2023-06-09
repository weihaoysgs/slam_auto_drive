#ifndef _ESKF_HPP__
#define _ESKF_HPP__

#include <glog/logging.h>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include "common/DataType.hpp"
#include "common/GNSS.hpp"

// 书中使用18维的ESKF，标量类型可以由S指定，默认取double
// 变量顺序：p, v, R, bg, ba, grav，与书本对应
template <typename S = double>
class ESKF
{
 public:
  using SO3 = Sophus::SO3<S>;
  using Vec3T = Eigen::Matrix<S, 3, 1>;
  using Vec18T = Eigen::Matrix<S, 18, 1>;
  using Mat3T = Eigen::Matrix<S, 3, 3>;
  using Mat18T = Eigen::Matrix<S, 18, 18>;
  using MotionNoiseT = Eigen::Matrix<S, 18, 18>;  // 运动噪声类型
  using OdomNoiseT = Eigen::Matrix<S, 3, 3>;      // 里程计噪声类型
  using GnssNoiseT = Eigen::Matrix<S, 6, 6>;      // GNSS噪声类型
  struct Options
  {
    Options() = default;

    // IMU 测量与零偏参数
    double imu_dt_ = 0.01;  // IMU测量间隔
    // NOTE IMU噪声项都为离散时间，不需要再乘dt，可以由初始化器指定IMU噪声
    double gyro_var_ = 1e-5;  // 陀螺测量标准差（方差的开根号）
    double acce_var_ = 1e-2;  // 加计测量标准差
    double bias_gyro_var_ = 1e-6;  // 陀螺零偏游走标准差
    double bias_acce_var_ = 1e-4;  // 加计零偏游走标准差

    double gnss_pos_noise_ = 0.1;                 // GNSS位置噪声
    double gnss_height_noise_ = 0.1;              // GNSS高度噪声
    double gnss_ang_noise_ = 1.0 * M_PI / 180.0;  // GNSS旋转噪声,存储的是弧度

    bool update_bias_gyro_ = true;  // 是否更新陀螺bias
    bool update_bias_acce_ = true;  // 是否更新加计bias
  };

  ESKF(Options option = Options()) : options_(option) { BuildNoise(option); }
  void SetInitialConditions(const Options& options, const Vec3T& init_bg,
                            const Vec3T& init_ba,
                            const Vec3T& gravity = Vec3T(0, 0, -9.8));
  bool Predict(const IMU& imu);
  bool ObserveGPS(const GNSS& gnss);
  bool ObserveSE3(const SE3& pose, double trans_noise, double angle_noise);
  void UpdateAndReset();
  void ProjectCov();

 private:
  void BuildNoise(const Options& option);

 private:
  Vec3T bg_ = Vec3T::Zero();
  Vec3T ba_ = Vec3T::Zero();
  Vec3T g_{0.0, 0.0, -9.81};

  // 运动方程噪声矩阵
  MotionNoiseT Q_ = MotionNoiseT::Zero();
  // 观测方程噪声矩阵
  GnssNoiseT gnss_noise_ = GnssNoiseT::Zero();
  // 配置参数
  Options options_;
  // 协方差矩阵
  Mat18T cov_ = Mat18T::Identity();

  Vec3T p_ = Vec3T::Zero();
  Vec3T v_ = Vec3T::Zero();
  SO3 R_;

  // 误差状态
  Vec18T dx_ = Vec18T::Zero();

  bool first_gnss_ = true;  // 是否为第一个gnss数据

  // 当前时间
  double current_time_ = 0.0;
};

using ESKFD = ESKF<double>;
using ESKFF = ESKF<float>;

template <typename S>
void ESKF<S>::BuildNoise(const Options& option)
{
  double ev = option.acce_var_;
  double et = option.gyro_var_;
  double eg = option.bias_gyro_var_;
  double ea = option.bias_acce_var_;

  double ev2 = ev;  // * ev;
  double et2 = et;  // * et;
  double eg2 = eg;  // * eg;
  double ea2 = ea;  // * ea;

  // 设置运动方程的噪声矩阵
  // p,v,r,bg,ba,g
  Q_.diagonal() << 0, 0, 0, ev2, ev2, ev2, et2, et2, et2, eg2, eg2, eg2, ea2,
      ea2, ea2, 0, 0, 0;

  // 设置GNSS的噪声矩阵
  // (x,y,height,R)
  double gp2 = option.gnss_pos_noise_ * option.gnss_pos_noise_;
  double gh2 = option.gnss_height_noise_ * option.gnss_height_noise_;
  double ga2 = option.gnss_ang_noise_ * option.gnss_ang_noise_;
  gnss_noise_.diagonal() << gp2, gp2, gh2, ga2, ga2, ga2;
}

template <typename S>
void ESKF<S>::SetInitialConditions(const Options& options, const Vec3T& init_bg,
                                   const Vec3T& init_ba, const Vec3T& gravity)
{
  this->BuildNoise(options);
  options_ = options;
  this->bg_ = init_bg;
  this->ba_ = init_ba;
  this->g_ = gravity;
  cov_ = Mat18T::Identity() * 1e-4;
}

template <typename S>
bool ESKF<S>::Predict(const IMU& imu)
{
  assert(imu.timestamp_ > current_time_);
  double dt = imu.timestamp_ - current_time_;
  // 这个阈值可以自己设置
  if (dt > 5 * options_.imu_dt_ || dt < 0)
  {
    LOG(ERROR) << "Imu data is not continue";
    current_time_ = imu.timestamp_;
    return false;
  }

  // 通过IMU进行航迹推算，nominal state 递增
  Vec3T new_p = p_ + v_ * dt + 0.5 * (R_ * (imu.acce_ - ba_)) * dt * dt +
                0.5 * g_ * dt * dt;
  Vec3T new_v = v_ + R_ * (imu.acce_ - ba_) * dt + g_ * dt;
  SO3 new_R = R_ * SO3::exp((imu.gyro_ - bg_) * dt);

  R_ = new_R;
  v_ = new_v;
  p_ = new_p;
  // 其余状态维度不变

  Mat18T F = Mat18T::Identity();                          // 主对角线
  F.template block<3, 3>(0, 3) = Mat3T::Identity() * dt;  // p 对 v
  F.template block<3, 3>(3, 6) =
      -R_.matrix() * SO3::hat(imu.acce_ - ba_) * dt;       // v对theta
  F.template block<3, 3>(3, 12) = -R_.matrix() * dt;       // v 对 ba
  F.template block<3, 3>(3, 15) = Mat3T::Identity() * dt;  // v 对 g
  F.template block<3, 3>(6, 6) =
      SO3::exp(-(imu.gyro_ - bg_) * dt).matrix();          // theta 对 theta
  F.template block<3, 3>(6, 9) = -Mat3T::Identity() * dt;  // theta 对 bg

  // mean and cov prediction
  dx_ = F * dx_;
  cov_ = F * cov_.eval() * F.transpose() + Q_;
  current_time_ = imu.timestamp_;
  return true;
}

template <typename S>
bool ESKF<S>::ObserveGPS(const GNSS& gnss)
{
  assert(gnss.unix_time_ >= current_time_);
  if (first_gnss_)
  {
    first_gnss_ = false;
    R_ = gnss.utm_pose_.so3();
    p_ = gnss.utm_pose_.translation();
    current_time_ = gnss.unix_time_;
    return true;
  }

  // 只有航向角有效的数据才会进行使用
  assert(gnss.heading_valid_);

  ObserveSE3(gnss.utm_pose_, options_.gnss_pos_noise_,
             options_.gnss_ang_noise_);

  current_time_ = gnss.unix_time_;
}

template <typename S>
bool ESKF<S>::ObserveSE3(const SE3& pose, double trans_noise,
                         double angle_noise)
{
  Eigen::Matrix<S, 6, 18> H = Eigen::Matrix<S, 6, 18>::Zero();
  H.template block<3, 3>(0, 0) = Mat3T::Identity();
  H.template block<3, 3>(3, 6) = Mat3T::Identity();

  Vec6d noise_vec;
  noise_vec << trans_noise, trans_noise, trans_noise, angle_noise, angle_noise,
      angle_noise;

  // 观测方程的噪声矩阵
  Mat6d V = noise_vec.asDiagonal();

  Eigen::Matrix<S, 18, 6> K =
      cov_ * H.transpose() * (H * cov_ * H.transpose() + V).inverse();

  Vec6d innov = Vec6d::Zero();
  innov.template head<3>() = (pose.translation() - p_);
  innov.template tail<3>() = (R_.inverse() * pose.so3()).log();

  // 得到观测值
  dx_ = K * innov;

  // 更新状态变量协方差
  cov_ = (Mat18T::Identity() - K * H) * cov_;

  return true;
}

template <typename S>
void ESKF<S>::UpdateAndReset()
{
  p_ += dx_.template block<3, 1>(0, 0);
  v_ += dx_.template block<3, 1>(3, 0);
  R_ = R_ * SO3::exp(dx_.template block<3, 1>(6, 0));

  if (options_.update_bias_gyro_)
  {
    bg_ += dx_.template block<3, 1>(9, 0);
  }

  if (options_.update_bias_acce_)
  {
    ba_ += dx_.template block<3, 1>(12, 0);
  }

  g_ += dx_.template block<3, 1>(15, 0);

  ProjectCov();

  dx_.setZero();
}

template <typename S>
void ESKF<S>::ProjectCov()
{
  Mat18T J = Mat18T::Identity();
  J.template block<3, 3>(6, 6) =
      Mat3T::Identity() - 0.5 * SO3::hat(dx_.template block<3, 1>(6, 0));
  cov_ = J * cov_ * J.transpose();
}

#endif  //_ESKF_HPP__