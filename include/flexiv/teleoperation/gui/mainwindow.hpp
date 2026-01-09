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
#include <flexiv/tdk/transparent_cartesian_teleop_lan.hpp>

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

  SM_TELEOP_INIT,
  SM_TELEOP_PREPROCESS,
  SM_TELEOP,
  SM_TELEOP_EXIT,
};

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

private slots:
  void update();

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
  // State machine for robot
  StateMachine state_machine_ = SM_DISCONNECT;

  // Thread for teleoperation
  std::unique_ptr<std::thread> thread_teleoperation_;

  void initUI();
};

} /* namespace teleoperation */
} /* namespace flexiv */

#endif // MAINWINDOW_HPP
