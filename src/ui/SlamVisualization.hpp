#ifndef _SLAM_VISUALZATION_HPP_
#define _SLAM_VISUALZATION_HPP_

#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <thread>
#include <string>
#include <memory>
#include <vector>
#include <sophus/se3.hpp>
#include <mutex>
#include <unistd.h>

class SlamVisualization
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  SlamVisualization() = default;
  ~SlamVisualization() = default;
  SlamVisualization(const SlamVisualization &) = delete;
  SlamVisualization &operator=(const SlamVisualization &) = delete;
  SlamVisualization(SlamVisualization &&) = delete;
  SlamVisualization &operator=(SlamVisualization &&) = delete;

public:
  void Init();
  bool DeInit();
  void RenderThread();
  void PushbackPosition(const Eigen::Vector3d &position)
  {
    positions_traj_.push_back(position);
  }
  void PushbackPose(const Sophus::SE3d &pose)
  {
    poses_.push_back(pose);
  }

private:
  void DrawCamera(const float scale = 1.) const;
  void DrawCameraWithPose() const;
  void DrawCoordinate() const;
  void DrawPositionTrajectory() const;

private:
  std::vector<Sophus::SE3d> poses_;
  std::vector<Eigen::Vector3d> positions_traj_;

private:
  pangolin::OpenGlRenderState s_cam_main_;
  pangolin::View view_main_;
  std::thread render_thread_;

  int window_width_ = 960;
  int window_height_ = 640;
  double left_pannel_width_ = 0.25;
  double right_pannel_width_ = 0.25;
  double one_plot_pannel_height_ = 0.33;
  std::string window_name_ = "slamVisualization";
};

#endif //_SLAM_VISUALZATION_HPP_
