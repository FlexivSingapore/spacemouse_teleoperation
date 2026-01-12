#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

/**
 * @file mainwindow.hpp
 * @date Oct 11, 2022
 * @author Zhan Fan Quek
 */

#include <memory>
#include <thread>
#include <QMainWindow>
#include <QTimer>
#include <spdlog/spdlog.h>

#include <flexiv/tdk/data.hpp>
#include <flexiv/tdk/device_teleop_lan.hpp>

#include "flexiv/teleoperation/data_types.hpp"
#include <flexiv/teleoperation/devices/HapticDeviceHandler.hpp>
#include <flexiv/teleoperation/devices/GenericHapticDevice.hpp>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

namespace flexiv {
namespace teleoperation {

/** State machine states */
enum StateMachine
{
  SM_DISCONNECT,
  SM_IDLE, ///< Idle mode, waiting for command

  SM_FREEDRIVE_INIT,
  SM_FREEDRIVE_CALISENSOR,     ///< Run force sensor calibration
  SM_FREEDRIVE_WAITCALISENSOR, ///< Wait for force sensor calibration to finish
  SM_FREEDRIVE,
  SM_FREEDRIVE_EXIT, ///< Exiting freedrive

  SM_TELEOP_INIT,
  SM_TELEOP_CALIFORCESENSOR,
  SM_TELEOP_WAITCALISENSOR,
  SM_TELEOP_PREPROCESS,
  SM_TELEOP,
  SM_TELEOP_EXIT,

  SM_MOVEPOSE_INIT,
  SM_MOVEPOSE_RUNPLAN_WAITFORFINISH,
  SM_MOVEPOSE_EXIT
};

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

private slots:
  void update();

  void on_connect_rdk_button_clicked();
  void on_set_idle_button_clicked();
  void on_set_move_home_button_clicked();
  void on_set_freedrive_button_clicked();
  void on_set_teleoperation_button_clicked();

private:
  // User interface
  Ui::MainWindow* ui_;

  // Timer for periodic update
  QTimer timer_;

  // ================== Haptic device ========================
  // Handler for haptic devices
  HapticDeviceHandler device_handler_;

  // Shared pointer to haptic device interface managed by m_deviceHandle
  GenericHapticDevicePtr device_;

  // Haptic device specs
  HapticDeviceInfo device_info_;

  // Number of haptic device
  unsigned int device_count_ = 0;

  // Haptic device pose
  Pose device_pose_;

  // Haptic device velocity
  Vec6d device_velocity_ = Vec6d::Zero();

  // Haptic device linear velocity after filter
  Vec6d device_velocity_filtered_ = Vec6d::Zero();

  // ========================= Teleoperation =======================
  // Teleoperation handler
  std::shared_ptr<flexiv::tdk::DeviceTeleopLan> device_teleop_ptr_;

  // Teleoperation command ptr
  std::vector<std::shared_ptr<flexiv::tdk::MotionControlCmds>> teleop_cmd_vector_;
  std::shared_ptr<flexiv::tdk::MotionControlCmds> teleop_cmd_;

  // Robot pointer
  std::shared_ptr<flexiv::rdk::Robot> robot_ptr_ = nullptr;

  // State machine for robot
  StateMachine state_machine_ = SM_DISCONNECT;

  // Teleoperation commands
  std::array<double, flexiv::tdk::kPoseSize> teleoperation_cmd_pose_;
  std::array<double, flexiv::tdk::kCartDoF> teleoperation_cmd_vel_;
  std::array<double, flexiv::tdk::kCartDoF> teleoperation_cmd_acc_;

  // Teleoperation target TCP pose
  Pose teleoperation_target_pose_;

  // Teleoperation start TCP pose
  Pose teleoperation_start_pose_;

  // Teleoperation TCP pose offset
  Pose teleoperation_offset_pose;

  // Maximum and minimum offset position
  Eigen::Vector3d teleoperation_offset_max_;
  Eigen::Vector3d teleoperation_offset_min_;

  // Translational and rotational Cartesian stiffness
  double translational_stiffness_ = 400;
  double rotational_stiffness_ = 100;

  // Maximum contact force
  double max_contact_force_x_ = 5.0;
  double max_contact_force_y_ = 5.0;
  double max_contact_force_z_ = 5.0;

  // Translation and rotational scaling
  double translational_scaling_ = 1.0;
  double rotational_scaling_ = 1.0;

  // Display forces information
  int display_force_percentage_ = 0;
  double display_force_norm_ = 0;

  // Thread for teleoperation
  std::unique_ptr<std::thread> thread_teleoperation_;

  void InitUI();

  void UpdateRobotStatus(bool flag_connected, bool flag_operational);

  void RunTeleoperation();
};

} /* namespace teleoperation */
} /* namespace flexiv */

#endif // MAINWINDOW_HPP
