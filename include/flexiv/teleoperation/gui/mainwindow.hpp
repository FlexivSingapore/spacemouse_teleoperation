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
};

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

private slots:

private:
  // User interface
  Ui::MainWindow* m_ui;

  // Timer for periodic update
  QTimer m_timer;

  // Timer for state machine
  QTimer m_stateTimer;

  // ================== Haptic device ========================
  // Handler for haptic devices
  HapticDeviceHandler m_deviceHandler;

  // Shared pointer to haptic device interface managed by m_deviceHandle
  GenericHapticDevicePtr m_device;

  // Haptic device specs
  HapticDeviceInfo m_deviceInfo;

  // Number of haptic device
  unsigned int m_deviceCount = 0;

  // Haptic device pose
  Pose m_devicePose;

  // Haptic device velocity
  Vec6d m_deviceVel = Vec6d::Zero();

  // Haptic device linear velocity after filter
  Vec6d m_deviceVelFilt = Vec6d::Zero();

  void initUI();
};

} /* namespace teleoperation */
} /* namespace flexiv */

#endif // MAINWINDOW_HPP
