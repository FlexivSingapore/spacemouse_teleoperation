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

#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Mode.hpp>
#include <flexiv/Robot.hpp>
#include <flexiv/Utility.hpp>

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
  SM_FREEDRIVE_RUNPLAN,        ///< Run freedrive plan
  SM_FREEDRIVE,
  SM_FREEDRIVE_EXIT, ///< Exiting freedrive

  SM_TELEOP_INIT,
  SM_TELEOP_CALIFORCESENSOR,
  SM_TELEOP_WAITCALISENSOR,
  SM_TELEOP_PREPROCESS,
  SM_TELEOP,
  SM_TELEOP_SWEEP_INIT,
  SM_TELEOP_SWEEP_RUN,
  SM_TELEOP_SWEEP_EXIT,

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
  void updateStateMachine();
  void on_connectRdk_button_clicked();

  void on_setIdle_button_clicked();
  void on_setFreeDrive_button_clicked();
  void on_setMoveHome_button_clicked();
  void on_setMovePose1_button_clicked();
  void on_setMovePose2_button_clicked();
  void on_setTeleoperation_button_clicked();

  void on_virtualconstraints_posx_checkbox_clicked(bool checked);
  void on_virtualconstraints_posy_checkbox_clicked(bool checked);
  void on_virtualconstraints_posz_checkbox_clicked(bool checked);

  void on_virtualconstraints_rotx_checkbox_clicked(bool checked);
  void on_virtualconstraints_roty_checkbox_clicked(bool checked);
  void on_virtualconstraints_rotz_checkbox_clicked(bool checked);

  void on_motionTranslationSF_doubleSpinBox_valueChanged(double value);
  void on_motionRotationSF_doubleSpinBox_valueChanged(double value);
  void on_motionTranslationLimit_doubleSpinBox_valueChanged(double value);
  void on_motionRotationLimit_doubleSpinBox_valueChanged(double value);
  
  void on_maxForceNorm_doubleSpinBox_valueChanged(double value);

  void on_sweep_button_clicked();
  void on_rotate0_button_clicked();
  void on_rotatePos90_button_clicked();
  void on_rotateNeg90_button_clicked();

private:
  // User interface
  Ui::MainWindow* m_ui;

  // Rdk client
  std::shared_ptr<flexiv::Robot> m_robot;

  // Timer for periodic update
  QTimer m_timer;

  // Timer for state machine
  QTimer m_stateTimer;

  // State machine for robot
  StateMachine m_stateMachine = SM_DISCONNECT;

  // Thread for teleoperation
  std::unique_ptr<std::thread> m_threadTeleoperation;

  // Thread for automatic motion
  std::unique_ptr<std::thread> m_threadAutoMotion;

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

  // =============== Teleoperation ===============
  // Slave robot target TCP pose
  Pose m_targetTcpPose;

  // Slave robot initial TCP pose
  Pose m_slaveInitTcpPose;

  // Slave TCP pose offset
  Pose m_slaveOffsetTcpPose;

  // Slave sweep rotation offset
  Eigen::Matrix3d m_slaveSweepRotate = Eigen::Matrix3d::Identity();

  // Slave rotate z axis offset
  Eigen::Matrix3d m_slaveRotateTcpZAxis = Eigen::Matrix3d::Identity();

  // Offset maximum value
  Eigen::Vector3d m_tcpOffsetMax;

  // Offset minimum value
  Eigen::Vector3d m_tcpOffsetMin;

  // Translational Cartesian stiffness
  double m_translationalStiffness = 400;

  // Rotational Cartesian stiffness
  double m_rotationalStiffness = 100;

  // Maximum force norm
  double m_maxForceNorm = 5.0;

  // Constraint activation
  Vec6i m_virtualConstraint;

  // Translation velocity limit
  double m_translationVelLimit = 0.5;

  // Rotation velocity limit
  double m_rotationVelLimit = 0.6;

  // Translation scaling
  double m_translationScaling = 1.0;

  // Rotation scaling
  double m_rotationScaling = 1.0;

  // Display force percentage
  int m_forcePercentage = 0;

  // Display force norm
  double m_forceNorm = 0;

  // Plan to run for move pose
  std::string m_movePosePlanName = "";

  // Sweep start time
  std::chrono::steady_clock::time_point m_timeSweepStart;

  // Sweep current time
  std::chrono::steady_clock::time_point m_timeSweepCurrent;

  // Sweep curent duration
  double m_sweepTime;

  void initUI();

  /**
   * @brief Initialize Rdk client for Series 3 robot
   */
  void initRdkClient();

  /**
   * @brief Enable the robot
   */
  void enableRobot();

  /**
   * @brief Callback function to update robot rdk connection status
   * @param[in] flagConnected Robot rdk connection status flag
   * @param[in] flagOperational Robot rdk operational status flag
   */
  void updateRobotRdkConnStatus(bool flagConnected, bool flagOperational);

  void processConnectionError();

  void RunTeleoperation();

  Eigen::Matrix3d RotationAboutXAxis(double angle);

  Eigen::Matrix3d RotationAboutYAxis(double angle);

  Eigen::Matrix3d RotationAboutZAxis(double angle);
};

} /* namespace teleoperation */
} /* namespace flexiv */

#endif // MAINWINDOW_HPP
