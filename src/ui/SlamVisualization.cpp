#include "SlamVisualization.hpp"

void SlamVisualization::Init()
{
  pangolin::CreateWindowAndBind(window_name_, window_width_, window_height_);
  glEnable(GL_DEPTH_TEST);
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

  s_cam_main_ = pangolin::OpenGlRenderState(
      pangolin::ProjectionMatrix(window_width_, window_height_, 420, 420, 320, 320, 0.1, 1000),
      pangolin::ModelViewLookAt(5, 5, 5, 0, 0, 0, pangolin::AxisZ));

  render_thread_ = std::thread(&SlamVisualization::RenderThread, this);
  render_thread_.detach();
}

void SlamVisualization::DrawCamera(const float scale) const
{
  if (scale < 0)
  {
    std::cerr << "scale should be positive !\n";
    return;
  }

  const float w = 0.2 * scale;
  const float h = w * 0.75;
  const float z = w * 0.8;

  glLineWidth(2 * scale);
  glBegin(GL_LINES);
  glColor3f(0.0f, 1.0f, 1.0f);
  glVertex3f(0, 0, 0);
  glVertex3f(w, h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, h, z);
  glVertex3f(w, h, z);
  glVertex3f(w, -h, z);
  glVertex3f(-w, h, z);
  glVertex3f(-w, -h, z);
  glVertex3f(-w, h, z);
  glVertex3f(w, h, z);
  glVertex3f(-w, -h, z);
  glVertex3f(w, -h, z);
  glEnd();
  return;
}

void SlamVisualization::DrawCameraWithPose() const
{
  if (!poses_.size())
    return;
  std::cout << "size: " << poses_.size() << std::endl;
  for (size_t i = 0; i < poses_.size() - 1; i++)
  {
    Eigen::Matrix3d R = poses_[i].unit_quaternion().toRotationMatrix();
    Eigen::Vector3d pos = poses_[i].translation();
    glPushMatrix();
    std::vector<GLdouble> Twc = {R(0, 0), R(1, 0), R(2, 0), 0.,
                                 R(0, 1), R(1, 1), R(2, 1), 0.,
                                 R(0, 2), R(1, 2), R(2, 2), 0.,
                                 pos.x(), pos.y(), pos.z(), 1.};
    glMultMatrixd(Twc.data());
    DrawCamera();
    glPopMatrix();
  }
}

void SlamVisualization::DrawCoordinate() const
{
  glLineWidth(3);
  glBegin(GL_LINES);
  // X axis red
  glColor3f(0.8f, 0.0f, 0.0f); // color
  glVertex3f(0, 0, 0);         // start point
  glVertex3f(1, 0, 0);         // end point
  // Y axis green
  glColor3f(0.0f, 0.8f, 0.0f);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 1, 0);
  // Z axis blue
  glColor3f(0.0f, 0.0f, 0.8f);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 0, 1);
  glEnd();
}

void SlamVisualization::DrawPositionTrajectory() const
{
  if (!positions_traj_.size())
    return;
  glLineWidth(2);
  glBegin(GL_LINES);
  glColor3f(0.f, 1.f, 0.f);
  for (size_t i = 0; i < positions_traj_.size() - 1; i++)
  {
    glVertex3d(positions_traj_[i].x(), positions_traj_[i].y(), positions_traj_[i].z());
    glVertex3d(positions_traj_[i + 1].x(), positions_traj_[i + 1].y(), positions_traj_[i + 1].z());
  }
  glEnd();
}

void SlamVisualization::RenderThread()
{
  pangolin::BindToContext(window_name_);
  pangolin::Handler3D handle(s_cam_main_);
  view_main_ = pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, (static_cast<double>(window_width_) / static_cast<double>(window_height_))).SetHandler(&handle);

  pangolin::CreatePanel("leftPannel")
      .SetBounds(0.0, 1.0, 0.0, left_pannel_width_);
  pangolin::Var<bool> button1("leftPannel.button1", false, false);    // 按钮
  pangolin::Var<bool> checkbox1("leftPannel.checkbox1", false, true); // 选框
  pangolin::Var<bool> SAVE_WIN("leftPannel.save_win", false, false);  // 选框

  while (!pangolin::ShouldQuit())
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // 激活之前设定好的视窗对象（否则视窗内会保留上一帧的图形）
    view_main_.Activate(s_cam_main_);

    DrawCoordinate();
    // DrawCameraWithPose();
    DrawPositionTrajectory();

    pangolin::FinishFrame();
    usleep(5e3);
  }
}
