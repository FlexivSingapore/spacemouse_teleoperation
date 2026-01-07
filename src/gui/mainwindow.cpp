/**
 * @file mainwindow.cpp
 * @date Oct 11, 2022
 * @author Zhan Fan Quek
 */

#include <QHostAddress>

#include "flexiv/teleoperation/gui/mainwindow.hpp"
#include "./ui_mainwindow.h"

namespace {
constexpr char k_defaultFreeDrivePlan[] = "PLAN-FreeDriveAuto";
constexpr char k_defaultMoveHome[] = "Sixx_MoveHome-MainPlan";
constexpr char k_defaultMovePose1[] = "Sixx_MovePose1-MainPlan";
constexpr char k_defaultMovePose2[] = "Sixx_MovePose2-MainPlan";

constexpr double k_defaultTcpOffsetMaxX = 0.2;
constexpr double k_defaultTcpOffsetMinX = -0.2;
constexpr double k_defaultTcpOffsetMaxY = 0.2;
constexpr double k_defaultTcpOffsetMinY = -0.2;
constexpr double k_defaultTcpOffsetMaxZ = 0.2;
constexpr double k_defaultTcpOffsetMinZ = -0.2;

constexpr float k_fc = 10;
constexpr float k_wc = 5 * M_PI * k_fc;
}

namespace flexiv {
namespace teleoperation {

MainWindow::MainWindow(QWidget* parent)
: QMainWindow(parent)
, m_ui(new Ui::MainWindow)
{
  m_ui->setupUi(this);

  connect(&m_timer, SIGNAL(timeout()), this, SLOT(update()));
  m_timer.start(30);

  connect(&m_stateTimer, SIGNAL(timeout()), this, SLOT(updateStateMachine()));
  m_stateTimer.start(30);

  // Haptic Device Initialization
  //=============================================================================
  // Check if device is connected
  m_deviceCount = m_deviceHandler.getNumDevices();
  if (m_deviceCount == 0) {
    printf("No haptic device detected\n");
  } else if (m_deviceCount != 1) {
    printf("More than 1 haptic device detected\n");
  }

  // Print information of detected haptic devices
  for (unsigned int i = 0; i < m_deviceCount; i++) {
    m_deviceHandler.getDeviceSpecifications(m_deviceInfo, 0);
  }

  // Access the first device
  if (m_deviceCount > 0) {
    m_deviceHandler.getDevice(m_device, 0);

    m_device->open();

    if (!m_device->calibrate(true)) {
      printf("Haptic device calibration failed\n");
    }
  }

  // Setup UI
  // =============================================================================
  initUI();

  m_tcpOffsetMax.x() = k_defaultTcpOffsetMaxX;
  m_tcpOffsetMax.y() = k_defaultTcpOffsetMaxY;
  m_tcpOffsetMax.z() = k_defaultTcpOffsetMaxZ;
  m_tcpOffsetMin.x() = k_defaultTcpOffsetMinX;
  m_tcpOffsetMin.y() = k_defaultTcpOffsetMinY;
  m_tcpOffsetMin.z() = k_defaultTcpOffsetMinZ;

  m_virtualConstraint.setZero();
}

MainWindow::~MainWindow()
{
  if (m_stateMachine == SM_TELEOP) {
    m_stateMachine = SM_TELEOP_EXIT;

    while (m_stateMachine != SM_IDLE) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  delete m_ui;
}

void MainWindow::update()
{
  if (m_robot != nullptr && m_robot->isConnected()) {
    // Update robot state
    flexiv::RobotStates robotStates;
    m_robot->getRobotStates(robotStates);

    // Check whether E - stop is engaged if (m_rdkClient->isEstopReleased() == false)
    {
      updateRobotRdkConnStatus(true, false);
    }

    // Check robot status
    if (m_robot->isOperational(false)) {
      m_ui->rdkOperationalStatus_label->setText("Ready");
      m_ui->rdkOperationalStatus_label->setStyleSheet(
          QStringLiteral("QLabel{color: rgb(0, 255, 0);}"));
      updateRobotRdkConnStatus(true, true);
    } else if (m_robot->isBusy()) {
      m_ui->rdkOperationalStatus_label->setText("Busy");
      m_ui->rdkOperationalStatus_label->setStyleSheet(
          QStringLiteral("QLabel{color: rgb(255, 165, 0);}"));
    } else if (m_robot->isFault()) {
      m_ui->rdkOperationalStatus_label->setText("Fault");
      m_ui->rdkOperationalStatus_label->setStyleSheet(
          QStringLiteral("QLabel{color: rgb(255, 0, 0);}"));
      processConnectionError();
    } else if (m_robot->isRecoveryState()) {
      m_ui->rdkOperationalStatus_label->setText("Recovery");
      m_ui->rdkOperationalStatus_label->setStyleSheet(
          QStringLiteral("QLabel{color: rgb(0, 0, 255);}"));
    } else {
      m_ui->rdkOperationalStatus_label->setText("Not Enabled");
      m_ui->rdkOperationalStatus_label->setStyleSheet(
          QStringLiteral("QLabel{color: rgb(211, 211, 211);}"));
      processConnectionError();
      updateRobotRdkConnStatus(true, false);
    }
  } else {
    updateRobotRdkConnStatus(false, false);
  }

  // Update state machine status
  if (m_stateMachine == SM_DISCONNECT) {
    m_ui->robotMode_label->setText("DISCONNECTED");
  } else if (m_stateMachine == SM_IDLE) {
    m_ui->robotMode_label->setText("IDLE");
  } else if (m_stateMachine == SM_FREEDRIVE) {
    m_ui->robotMode_label->setText("FREE DRIVE");
  } else if (m_stateMachine == SM_TELEOP) {
    m_ui->robotMode_label->setText("TELEOP");
  } else if (m_stateMachine == SM_MOVEPOSE_RUNPLAN_WAITFORFINISH) {
    m_ui->robotMode_label->setText("MOVE_TO_POSE");
  }else {
    m_ui->robotMode_label->setText("Transiting...");
  }

  // Update force display
  if ((m_stateMachine == SM_TELEOP)
  ||(m_stateMachine == SM_TELEOP_SWEEP_INIT)
 ||(m_stateMachine == SM_TELEOP_SWEEP_RUN)
 ||(m_stateMachine == SM_TELEOP_SWEEP_EXIT)) {
    m_ui->force_lcdNumber->display(m_forceNorm);
    m_ui->force_progressBar->setValue(m_forcePercentage);
  }
}

void MainWindow::updateStateMachine()
{
  switch (m_stateMachine) {

    case SM_DISCONNECT: {
      if (m_robot != nullptr) {
        if (m_robot->isConnected()) {
          m_stateMachine = SM_IDLE;
        }
      }
      break;
    }

    case SM_IDLE: {
      break;
    }

    case SM_FREEDRIVE_INIT: {

      // Set operation mode to primitive execution mode
      m_robot->setMode(flexiv::Mode::NRT_PRIMITIVE_EXECUTION);
      m_stateMachine = SM_FREEDRIVE_CALISENSOR;

      break;
    }

    case SM_FREEDRIVE_CALISENSOR: {

      char cmd_string_zeroFTSensor[512];
      sprintf(cmd_string_zeroFTSensor,
          "ZeroFTSensor(dataCollectionTime=%5.4f, enableStaticCheck=1)", 0.1);
      std::string cmd = cmd_string_zeroFTSensor;
      m_robot->executePrimitive(cmd);

      m_stateMachine = SM_FREEDRIVE_WAITCALISENSOR;
      break;
    }

    case SM_FREEDRIVE_WAITCALISENSOR: {
      // Wait for primitive execution to complete
      if (flexiv::utility::parsePtStates(m_robot->getPrimitiveStates(), "reachedTarget") == "1") {
        m_stateMachine = SM_FREEDRIVE_RUNPLAN;
      }
      break;
    }

    case SM_FREEDRIVE_RUNPLAN: {
      // Set operation mode to plan execution mode
      m_robot->setMode(flexiv::Mode::NRT_PLAN_EXECUTION);

      // Robot run free drive
      m_robot->executePlan(k_defaultFreeDrivePlan);
      m_stateMachine = SM_FREEDRIVE;
      break;
    }

    case SM_FREEDRIVE: {
      break;
    }

    case SM_FREEDRIVE_EXIT: {

      // Set operation mode to Idle
      m_robot->setMode(flexiv::Mode::IDLE);
      m_stateMachine = SM_IDLE;
      break;
    }

    case SM_TELEOP_INIT: {

      // Set operation mode to primitive execution mode
      m_robot->setMode(flexiv::Mode::NRT_PRIMITIVE_EXECUTION);
      m_stateMachine = SM_TELEOP_CALIFORCESENSOR;

      break;
    }

    case SM_TELEOP_CALIFORCESENSOR: {

      char cmd_string_zeroFTSensor[512];
      sprintf(cmd_string_zeroFTSensor,
          "ZeroFTSensor(dataCollectionTime=%5.4f, enableStaticCheck=1)", 0.1);
      std::string cmd = cmd_string_zeroFTSensor;
      m_robot->executePrimitive(cmd);
      m_stateMachine = SM_TELEOP_WAITCALISENSOR;
      break;
    }

    case SM_TELEOP_WAITCALISENSOR: {
      // Wait for primitive execution to complete
      if (flexiv::utility::parsePtStates(m_robot->getPrimitiveStates(), "reachedTarget") == "1") {
        m_stateMachine = SM_TELEOP_PREPROCESS;
      }
      break;
    }

    case SM_TELEOP_PREPROCESS: {

      // ==== Pre-processing before running teleoperation ====
      // Initial Reference Pose
      //=============================================================================
      // Read new robot states
      flexiv::RobotStates robotStates;
      m_robot->getRobotStates(robotStates);

      // Save current robot TCP position as the first reference position
      for (size_t i = 0; i < k_cartPositionDofs; i++) {
        m_targetTcpPose.pos[i] = robotStates.tcpPose[i];
        m_slaveInitTcpPose.pos[i] = robotStates.tcpPose[i];
      }
      // Same for rotation
      Eigen::Quaterniond q(robotStates.tcpPose[3], robotStates.tcpPose[4], robotStates.tcpPose[5],
          robotStates.tcpPose[6]);
      m_targetTcpPose.rot = q.toRotationMatrix();
      m_slaveInitTcpPose.rot = q.toRotationMatrix();

      // Set the offset tcp pose to identity
      m_slaveOffsetTcpPose.pos = Eigen::Vector3d::Zero();
      m_slaveOffsetTcpPose.rot = Eigen::Matrix3d::Identity();

      // Set operation mode to NRT
      m_robot->setMode(flexiv::Mode::NRT_CARTESIAN_MOTION_FORCE);

      // Cartesian stiffness
      //=============================================================================
      // Set slave Cartesian stiffness
      m_translationalStiffness = m_ui->teleopTranslationalStiffness_doubleSpinBox->value();
      m_rotationalStiffness = m_ui->teleopRotationalStiffness_doubleSpinBox->value();
      std::vector<double> stiffness
          = {m_translationalStiffness, m_translationalStiffness, m_translationalStiffness,
              m_rotationalStiffness, m_rotationalStiffness, m_rotationalStiffness};
      m_robot->setCartesianStiffness(stiffness);
      m_ui->tabCartesianStiffness->setEnabled(false);

      // Workspace offset
      //=============================================================================
      m_tcpOffsetMax.x() = m_ui->maxXOffset_doubleSpinBox->value();
      m_tcpOffsetMax.y() = m_ui->maxYOffset_doubleSpinBox->value();
      m_tcpOffsetMax.z() = m_ui->maxZOffset_doubleSpinBox->value();

      m_tcpOffsetMin.x() = m_ui->minXOffset_doubleSpinBox->value();
      m_tcpOffsetMin.y() = m_ui->minYOffset_doubleSpinBox->value();
      m_tcpOffsetMin.z() = m_ui->minZOffset_doubleSpinBox->value();
      m_ui->tabWorkspace->setEnabled(false);

      m_maxForceNorm = m_ui->maxForceNorm_doubleSpinBox->value();
      std::vector<double> maxContactWrench{m_maxForceNorm, m_maxForceNorm, m_maxForceNorm,40,40,40};
      m_robot->setMaxContactWrench(maxContactWrench);

      // Setup thread to run NRT stream cartesian command
      m_threadTeleoperation
          = std::make_unique<std::thread>(std::bind(&MainWindow::RunTeleoperation, this));

      m_stateMachine = SM_TELEOP;

      break;
    }

    case SM_TELEOP: {
      break;
    }

    case SM_TELEOP_SWEEP_INIT:{

      m_timeSweepStart = std::chrono::steady_clock::now();
      m_stateMachine = SM_TELEOP_SWEEP_RUN;
      break;
    }

    case SM_TELEOP_SWEEP_RUN:{

      m_timeSweepCurrent = std::chrono::steady_clock::now();
      m_sweepTime = std::chrono::duration<double, std::milli>(m_timeSweepCurrent - m_timeSweepStart).count()/1000.0;
      if (m_sweepTime >= 20)
      {
        m_slaveSweepRotate = RotationAboutYAxis(0);

        m_stateMachine = SM_TELEOP_SWEEP_EXIT;
      }
      else
      {
        double angle = 0;
        if (m_sweepTime <= 5)
        {
          angle = (30.0 / 180.0 * M_PI) * m_sweepTime / 5.0;
        }
        else if (m_sweepTime <= 10)
        {
          angle = (30.0 / 180.0 * M_PI) * (1 - (m_sweepTime-5.0) / 5.0);
        }
        else if (m_sweepTime <= 15)
        {
          angle = (30.0 / 180.0 * M_PI) * (-(m_sweepTime-10.0) / 5.0);
        }
        else if (m_sweepTime <= 20)
        {
          angle = (30.0 / 180.0 * M_PI) * (-1+(m_sweepTime-15.0) / 5.0);
        }

        m_slaveSweepRotate = RotationAboutYAxis(angle);
      }

      break;
    }

    case SM_TELEOP_SWEEP_EXIT:{

      m_ui->automotion_groupBox->setEnabled(true);
      m_stateMachine = SM_TELEOP;
      break;
    }

    case SM_TELEOP_EXIT: {

      // Wait for thread to join
      m_threadTeleoperation->join();

      // ==== Post-processing after running teleoperation ====
      m_ui->tabCartesianStiffness->setEnabled(true);
      m_ui->tabWorkspace->setEnabled(true);

      // Set operation mode to Idle
      m_robot->setMode(flexiv::Mode::IDLE);

      m_stateMachine = SM_IDLE;
      break;
    }

    case SM_MOVEPOSE_INIT: {
      // Set operation mode to plan execution mode
      m_robot->setMode(flexiv::Mode::NRT_PLAN_EXECUTION);

      // Robot run free drive
      m_robot->executePlan(m_movePosePlanName);
      m_stateMachine = SM_MOVEPOSE_RUNPLAN_WAITFORFINISH;
      break;
    }

    case SM_MOVEPOSE_RUNPLAN_WAITFORFINISH: {
      if (!m_robot->isBusy()) {
        m_stateMachine = SM_MOVEPOSE_EXIT;
      }
      break;
    }

    case SM_MOVEPOSE_EXIT: {

      m_ui->setIdle_button->setEnabled(false);
      m_ui->setFreeDrive_button->setEnabled(true);
      m_ui->setMoveHome_button->setEnabled(true);
      m_ui->setMovePose1_button->setEnabled(true);
      m_ui->setMovePose2_button->setEnabled(true);
      m_ui->setTeleoperation_button->setEnabled(true);

      // Set operation mode to Idle
      m_robot->setMode(flexiv::Mode::IDLE);
      m_stateMachine = SM_IDLE;
      break;
    }

    default: {
      break;
    }
  }
}

void MainWindow::on_connectRdk_button_clicked()
{
  auto robotIP = m_ui->robot_ip_lineEdit->text();
  QHostAddress addressRobot(robotIP);
  if (QAbstractSocket::IPv4Protocol != addressRobot.protocol()) {
    return;
  }

  auto localIP = m_ui->local_ip_lineEdit->text();
  QHostAddress addressLocal(localIP);
  if (QAbstractSocket::IPv4Protocol != addressLocal.protocol()) {
    return;
  }

  if (m_robot == nullptr || !m_robot->isConnected()) {
    initRdkClient();
  } else {
    try {
      // Enable robot server
      enableRobot();
    } catch (const std::exception& e) {
      printf(e.what());
    }
  }
}

void MainWindow::on_setIdle_button_clicked()
{
  if (m_stateMachine == SM_FREEDRIVE) {
    m_stateMachine = SM_FREEDRIVE_EXIT;

    m_ui->setIdle_button->setEnabled(false);
    m_ui->setFreeDrive_button->setEnabled(true);
    m_ui->setMoveHome_button->setEnabled(true);
    m_ui->setMovePose1_button->setEnabled(true);
    m_ui->setMovePose2_button->setEnabled(true);
    m_ui->setTeleoperation_button->setEnabled(true);
  } else if (m_stateMachine == SM_TELEOP) {
    m_stateMachine = SM_TELEOP_EXIT;

    m_ui->setIdle_button->setEnabled(false);
    m_ui->setFreeDrive_button->setEnabled(true);
    m_ui->setMoveHome_button->setEnabled(true);
    m_ui->setMovePose1_button->setEnabled(true);
    m_ui->setMovePose2_button->setEnabled(true);
    m_ui->setTeleoperation_button->setEnabled(true);

    m_ui->automotion_groupBox->setEnabled(false);
  } 
}

void MainWindow::on_setFreeDrive_button_clicked()
{
  if (m_stateMachine == SM_IDLE) {
    m_stateMachine = SM_FREEDRIVE_INIT;

    m_ui->setIdle_button->setEnabled(true);
    m_ui->setFreeDrive_button->setEnabled(false);
    m_ui->setMoveHome_button->setEnabled(false);
    m_ui->setMovePose1_button->setEnabled(false);
    m_ui->setMovePose2_button->setEnabled(false);
    m_ui->setTeleoperation_button->setEnabled(false);
  }
}

void MainWindow::on_setMoveHome_button_clicked()
{
  if (m_stateMachine == SM_IDLE) {
    m_movePosePlanName = std::string(k_defaultMoveHome);
    m_stateMachine = SM_MOVEPOSE_INIT;

    m_ui->setIdle_button->setEnabled(false);
    m_ui->setFreeDrive_button->setEnabled(false);
    m_ui->setMoveHome_button->setEnabled(false);
    m_ui->setMovePose1_button->setEnabled(false);
    m_ui->setMovePose2_button->setEnabled(false);
    m_ui->setTeleoperation_button->setEnabled(false);
  }
}

void MainWindow::on_setMovePose1_button_clicked()
{
  if (m_stateMachine == SM_IDLE) {
    m_movePosePlanName = std::string(k_defaultMovePose1);
    m_stateMachine = SM_MOVEPOSE_INIT;

    m_ui->setIdle_button->setEnabled(false);
    m_ui->setFreeDrive_button->setEnabled(false);
    m_ui->setMoveHome_button->setEnabled(false);
    m_ui->setMovePose1_button->setEnabled(false);
    m_ui->setMovePose2_button->setEnabled(false);
    m_ui->setTeleoperation_button->setEnabled(false);
  }
}

void MainWindow::on_setMovePose2_button_clicked()
{
  if (m_stateMachine == SM_IDLE) {
    m_movePosePlanName = std::string(k_defaultMovePose2);
    m_stateMachine = SM_MOVEPOSE_INIT;

    m_ui->setIdle_button->setEnabled(false);
    m_ui->setFreeDrive_button->setEnabled(false);
    m_ui->setMoveHome_button->setEnabled(false);
    m_ui->setMovePose1_button->setEnabled(false);
    m_ui->setMovePose2_button->setEnabled(false);
    m_ui->setTeleoperation_button->setEnabled(false);
  }
}

void MainWindow::on_setTeleoperation_button_clicked()
{
  if (m_stateMachine == SM_IDLE) {

    if (m_deviceCount > 0) {
      m_stateMachine = SM_TELEOP_INIT;
      m_ui->setIdle_button->setEnabled(true);
      m_ui->setFreeDrive_button->setEnabled(false);
      m_ui->setMoveHome_button->setEnabled(false);
      m_ui->setMovePose1_button->setEnabled(false);
      m_ui->setMovePose2_button->setEnabled(false);
      m_ui->setTeleoperation_button->setEnabled(false);

      m_ui->automotion_groupBox->setEnabled(true);
    } else {
      printf("No haptic device available. Unable to run teleoperation\n");
    }
  }
}

void MainWindow::on_virtualconstraints_posx_checkbox_clicked(bool checked)
{
  if (checked) {
    m_ui->virtualconstraints_posx_label->setText("Activated");
    m_virtualConstraint(0) = 1;
  } else {
    m_ui->virtualconstraints_posx_label->setText("Deactivated");
    m_virtualConstraint(0) = 0;
  }
}

void MainWindow::on_virtualconstraints_posy_checkbox_clicked(bool checked)
{
  if (checked) {
    m_ui->virtualconstraints_posy_label->setText("Activated");
    m_virtualConstraint(1) = 1;
  } else {
    m_ui->virtualconstraints_posy_label->setText("Deactivated");
    m_virtualConstraint(1) = 0;
  }
}

void MainWindow::on_virtualconstraints_posz_checkbox_clicked(bool checked)
{
  if (checked) {
    m_ui->virtualconstraints_posz_label->setText("Activated");
    m_virtualConstraint(2) = 1;
  } else {
    m_ui->virtualconstraints_posz_label->setText("Deactivated");
    m_virtualConstraint(2) = 0;
  }
}

void MainWindow::on_virtualconstraints_rotx_checkbox_clicked(bool checked)
{
  if (checked) {
    m_ui->virtualconstraints_rotx_label->setText("Activated");
    m_virtualConstraint(3) = 1;
  } else {
    m_ui->virtualconstraints_rotx_label->setText("Deactivated");
    m_virtualConstraint(3) = 0;
  }
}

void MainWindow::on_virtualconstraints_roty_checkbox_clicked(bool checked)
{
  if (checked) {
    m_ui->virtualconstraints_roty_label->setText("Activated");
    m_virtualConstraint(4) = 1;
  } else {
    m_ui->virtualconstraints_roty_label->setText("Deactivated");
    m_virtualConstraint(4) = 0;
  }
}

void MainWindow::on_virtualconstraints_rotz_checkbox_clicked(bool checked)
{
  if (checked) {
    m_ui->virtualconstraints_rotz_label->setText("Activated");
    m_virtualConstraint(5) = 1;
  } else {
    m_ui->virtualconstraints_rotz_label->setText("Deactivated");
    m_virtualConstraint(5) = 0;
  }
}

void MainWindow::on_motionTranslationSF_doubleSpinBox_valueChanged(double value)
{
  m_translationScaling = value;
}

void MainWindow::on_motionRotationSF_doubleSpinBox_valueChanged(double value)
{
  m_rotationScaling = value;
}

void MainWindow::on_motionTranslationLimit_doubleSpinBox_valueChanged(double value)
{
  m_translationVelLimit = value;
}

void MainWindow::on_motionRotationLimit_doubleSpinBox_valueChanged(double value)
{
  m_rotationVelLimit = value;
}

void MainWindow::on_maxForceNorm_doubleSpinBox_valueChanged(double value)
{
  m_maxForceNorm = value;
}

void MainWindow::on_sweep_button_clicked()
{
  if (m_stateMachine == SM_TELEOP)
  {
    m_ui->automotion_groupBox->setEnabled(false);
    m_stateMachine = SM_TELEOP_SWEEP_INIT;
  }
}

void MainWindow::on_rotate0_button_clicked()
{
  m_slaveRotateTcpZAxis = RotationAboutZAxis(0);
  m_ui->rotate0_button->setEnabled(false);
  m_ui->rotatePos90_button->setEnabled(true);
  m_ui->rotateNeg90_button->setEnabled(true);

  m_ui->setIdle_button->setEnabled(true);
}

void MainWindow::on_rotatePos90_button_clicked()
{
  m_slaveRotateTcpZAxis = RotationAboutZAxis(M_PI/2);
  m_ui->rotate0_button->setEnabled(true);
  m_ui->rotatePos90_button->setEnabled(false);
  m_ui->rotateNeg90_button->setEnabled(false);

  m_ui->setIdle_button->setEnabled(false);
}

void MainWindow::on_rotateNeg90_button_clicked()
{
  m_slaveRotateTcpZAxis = RotationAboutZAxis(-M_PI/2);
  m_ui->rotate0_button->setEnabled(true);
  m_ui->rotatePos90_button->setEnabled(false);
  m_ui->rotateNeg90_button->setEnabled(false);

  m_ui->setIdle_button->setEnabled(false);
}

void MainWindow::initUI()
{
  m_ui->maxXOffset_doubleSpinBox->setValue(k_defaultTcpOffsetMaxX);
  m_ui->maxYOffset_doubleSpinBox->setValue(k_defaultTcpOffsetMaxY);
  m_ui->maxZOffset_doubleSpinBox->setValue(k_defaultTcpOffsetMaxZ);
  m_ui->minXOffset_doubleSpinBox->setValue(k_defaultTcpOffsetMinX);
  m_ui->minYOffset_doubleSpinBox->setValue(k_defaultTcpOffsetMinY);
  m_ui->minZOffset_doubleSpinBox->setValue(k_defaultTcpOffsetMinZ);
}

void MainWindow::initRdkClient()
{
  // Start RDK client
  try {
    // Instantiate robot interface
    m_robot = std::make_shared<flexiv::Robot>(m_ui->robot_ip_lineEdit->text().toStdString(),
        m_ui->local_ip_lineEdit->text().toStdString());

    // Enable the robot
    enableRobot();

  } catch (const std::exception& e) {
    printf(e.what());
  }
}

void MainWindow::enableRobot()
{
  if (m_robot != nullptr) {
    try {
      // Clear fault on robot server if any
      if (m_robot->isFault()) {
        printf("[RDK warning]: Fault occurred on robot server, trying to clear ...\n");
        // Try to clear the fault
        m_robot->clearFault();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        // Check again
        if (m_robot->isFault()) {
          printf("[RDK error]: Fault cannot be cleared, exiting ...\n");
          return;
        }
        printf("[RDK info]: Fault on robot server is cleared\n");
      }

      // Enable robot server
      m_robot->enable();
      m_robot->connect();

      // Wait for the robot to become operational
      int secondsWaited = 0;
      do {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (++secondsWaited == 10) {
          printf(
              "[RDK error]: Robot is not operational. Please check that the E-stop is released, "
              "the robot is in Auto Remote mode and has no fault.\n");
          break;
        }
      } while (!m_robot->isOperational(false));

      if (m_robot->isOperational(false)) {
        printf("[RDK info]: Robot is operational\n");
      }

    } catch (const std::exception& e) {
      printf(e.what());
    }

    // Set control mode for robot
    m_robot->setMode(flexiv::Mode::IDLE);
    m_stateMachine = SM_IDLE;

    m_ui->robotMode_groupBox->setEnabled(true);
  }
}

void MainWindow::updateRobotRdkConnStatus(bool flagConnected, bool flagOperational)
{
  if (flagConnected) {
    m_ui->rdkConnectionStatus_label->setText("Connected");
    m_ui->rdkConnectionStatus_label->setStyleSheet(
        QStringLiteral("QLabel{color: rgb(0, 255, 0);}"));
    if (flagOperational) {
      m_ui->connectRdk_button->setText("RDK Connected/Enabled");
      m_ui->connectRdk_button->setEnabled(false);
    } else {
      m_ui->connectRdk_button->setText("Enable Robot");
      m_ui->connectRdk_button->setEnabled(true);
    }

  } else {
    m_ui->rdkConnectionStatus_label->setText("Disconnected");
    m_ui->rdkConnectionStatus_label->setStyleSheet(
        QStringLiteral("QLabel{color: rgb(255, 0, 0);}"));
    m_ui->connectRdk_button->setText("Connect");
    m_ui->connectRdk_button->setEnabled(true);

    m_ui->robotMode_groupBox->setEnabled(false);
    m_ui->setIdle_button->setEnabled(false);
  }
}

void MainWindow::processConnectionError()
{
  m_stateMachine = SM_DISCONNECT;
  m_ui->robotMode_groupBox->setEnabled(false);
}

void MainWindow::RunTeleoperation()
{
  while ((m_stateMachine == SM_TELEOP) 
  || (m_stateMachine == SM_TELEOP_SWEEP_INIT)
  || (m_stateMachine == SM_TELEOP_SWEEP_RUN)
  || (m_stateMachine == SM_TELEOP_SWEEP_EXIT)) {
    // Run teleoperation code

    // Get current button state
    bool button0 = false;
    m_device->getUserSwitch(0, button0);

    bool button1 = false;
    m_device->getUserSwitch(1, button1);

    
    if (button0)
    {
      m_virtualConstraint(0) = 0;
      m_virtualConstraint(1) = 0;
      m_virtualConstraint(2) = 0;
      m_virtualConstraint(3) = 1;
      m_virtualConstraint(4) = 1;
      m_virtualConstraint(5) = 1;
    }
    else if (button1)
    {
      m_virtualConstraint(0) = 1;
      m_virtualConstraint(1) = 1;
      m_virtualConstraint(2) = 1;
      m_virtualConstraint(3) = 0;
      m_virtualConstraint(4) = 0;
      m_virtualConstraint(5) = 0;
    }


    // Read current robot state
    RobotStates robotStates;
    m_robot->getRobotStates(robotStates);

    // Read new haptic device states
    m_device->getPosition(m_devicePose.pos);
    m_device->getRotation(m_devicePose.rot);
    m_device->getLinearVelocity(m_deviceVel.head(k_cartPositionDofs));
    m_device->getAngularVelocity(m_deviceVel.tail(k_cartOrientationDofs));

    // ============================
    // Display force value
    // ============================
    Eigen::Vector3d forceBase(robotStates.extWrenchInBase[0], robotStates.extWrenchInBase[1],
        robotStates.extWrenchInBase[2]);
    m_forceNorm = forceBase.norm();

    double percentageForceNorm = forceBase.norm() / m_maxForceNorm * 100;
    if (percentageForceNorm > 100) {
      percentageForceNorm = 100;
    }
    m_forcePercentage = static_cast<int>(percentageForceNorm);

    // ============================
    // Filter velocity
    // ============================
    // Input filter on device signals to reject disturbance feed-forwarding
    for (size_t i = 0; i < k_cartPoseDofs; i++) {
      m_deviceVelFilt[i] = 0.8 * m_deviceVelFilt[i] + 0.2 * m_deviceVel[i];
    }

    auto linearVelFilt = m_deviceVelFilt.head(k_cartPositionDofs);
    auto angularVelFilt = m_deviceVelFilt.tail(k_cartOrientationDofs);

    // Apply virtual constraint
    if (m_virtualConstraint(0)) {
      linearVelFilt.x() = 0;
    }
    if (m_virtualConstraint(1)) {
      linearVelFilt.y() = 0;
    }
    if (m_virtualConstraint(2)) {
      linearVelFilt.z() = 0;
    }
    if (m_virtualConstraint(3)) {
      angularVelFilt.x() = 0;
    }
    if (m_virtualConstraint(4)) {
      angularVelFilt.y() = 0;
    }
    if (m_virtualConstraint(5)) {
      angularVelFilt.z() = 0;
    }

    // ============================
    // Limit velocity due to force
    // ============================
    // Get the current robot position
    Eigen::Vector3d robotCurrentPos(
        robotStates.tcpPose[0], robotStates.tcpPose[1], robotStates.tcpPose[2]);

    // Get the difference between target position and current position
    Eigen::Vector3d robotTargetPos(
        m_targetTcpPose.pos.x(), m_targetTcpPose.pos.y(), m_targetTcpPose.pos.z());
    Eigen::Vector3d diffPos = robotCurrentPos - robotTargetPos;

    double diffPosNorm = diffPos.norm();
    double boundaryNorm = m_maxForceNorm / m_translationalStiffness;

    if (diffPosNorm >= boundaryNorm) {
      Eigen::Vector3d diffPosAxis = diffPos / diffPos.norm();

      // Only accept linear velocity that is pointing to the robot actual position
      double normInDirTowardsRobot = linearVelFilt.dot(diffPosAxis);

      // Set the slave offset position to the boundary
      //m_slaveOffsetTcpPose.pos
      //    = (robotCurrentPos + boundaryNorm * diffPosAxis * -1.0) - m_slaveInitTcpPose.pos;

      if (normInDirTowardsRobot > 0) {
        linearVelFilt = normInDirTowardsRobot * diffPosAxis;
      } else {
        linearVelFilt.setZero();
      }
    }

    /*
    printf("Device linear velocity: %5.4f\t%5.4f\t%5.4f\n", linearVelFilt[0], linearVelFilt[1],
        linearVelFilt[2]);
    printf("Device angular velocity: %5.4f\t%5.4f\t%5.4f\n", angularVelFilt[0], angularVelFilt[1],
        angularVelFilt[2]);
    */

    // ============================
    // Integrate linear velocity
    // ============================
    // Integrate linear velocity to get new TCP position
    m_slaveOffsetTcpPose.pos.x()
        += linearVelFilt(0) * k_loopPeriod * -1.0 * 0.1 * m_translationScaling;
    m_slaveOffsetTcpPose.pos.y()
        += linearVelFilt(1) * k_loopPeriod * -1.0 * 0.1 * m_translationScaling;
    m_slaveOffsetTcpPose.pos.z() += linearVelFilt(2) * k_loopPeriod * 0.1 * m_translationScaling;

    // Limit slave Tcp pose offset to within a bound
    if (m_slaveOffsetTcpPose.pos.x() >= m_tcpOffsetMax.x()) {
      m_slaveOffsetTcpPose.pos.x() = m_tcpOffsetMax.x();
    } else if (m_slaveOffsetTcpPose.pos.x() <= m_tcpOffsetMin.x()) {
      m_slaveOffsetTcpPose.pos.x() = m_tcpOffsetMin.x();
    }

    if (m_slaveOffsetTcpPose.pos.y() >= m_tcpOffsetMax.y()) {
      m_slaveOffsetTcpPose.pos.y() = m_tcpOffsetMax.y();
    } else if (m_slaveOffsetTcpPose.pos.y() <= m_tcpOffsetMin.y()) {
      m_slaveOffsetTcpPose.pos.y() = m_tcpOffsetMin.y();
    }

    if (m_slaveOffsetTcpPose.pos.z() >= m_tcpOffsetMax.z()) {
      m_slaveOffsetTcpPose.pos.z() = m_tcpOffsetMax.z();
    } else if (m_slaveOffsetTcpPose.pos.z() <= m_tcpOffsetMin.z()) {
      m_slaveOffsetTcpPose.pos.z() = m_tcpOffsetMin.z();
    }

    // ============================
    // Integrate angular velocity
    // ============================
    // Integrate angular velocity to get new TCP orientation
    Eigen::AngleAxisd angleAxis;
    double angVelNorm = angularVelFilt.norm();
    if (angVelNorm <= k_epsilon) {
      angleAxis.angle() = 0;
      angleAxis.axis() = {0, 0, 1};
    } else {
      angleAxis.angle() = angVelNorm * k_loopPeriod * 0.1 * m_rotationScaling;
      Eigen::Vector3d axis = angularVelFilt / angVelNorm;
      angleAxis.axis().x() = axis.x() * -1.0;
      angleAxis.axis().y() = axis.y() * -1.0;
      angleAxis.axis().z() = axis.z();
    }
    m_slaveOffsetTcpPose.rot = angleAxis.toRotationMatrix() * m_slaveOffsetTcpPose.rot;

    // Limit slave rotation
    Eigen::AngleAxisd slaveRotAngleAxis;
    slaveRotAngleAxis.fromRotationMatrix(m_slaveOffsetTcpPose.rot);

    if (slaveRotAngleAxis.angle() >= M_PI / 4) {
      slaveRotAngleAxis.angle() = M_PI / 4;
    }
    m_slaveOffsetTcpPose.rot = slaveRotAngleAxis.toRotationMatrix();

    // ============================
    // Calculate the target pose
    // ============================
    // Set target TCP pose
    m_targetTcpPose.pos = m_slaveInitTcpPose.pos + m_slaveOffsetTcpPose.pos;
    m_targetTcpPose.rot = m_slaveOffsetTcpPose.rot * m_slaveInitTcpPose.rot * m_slaveRotateTcpZAxis * m_slaveSweepRotate;

    // ============================
    // Send target tcp pose to robot
    // ============================
    // Form pose vector
    std::vector<double> targetTcpPose(7);
    for (size_t i = 0; i < 3; i++) {
      targetTcpPose[i] = m_targetTcpPose.pos[i];
    }
    Eigen::Quaterniond targetQuat(m_targetTcpPose.rot);
    targetTcpPose[3] = targetQuat.w();
    targetTcpPose[4] = targetQuat.x();
    targetTcpPose[5] = targetQuat.y();
    targetTcpPose[6] = targetQuat.z();

    //printf("%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\n", targetTcpPose[0],
    //targetTcpPose[1],
    //     targetTcpPose[2], targetTcpPose[3], targetTcpPose[4], targetTcpPose[5],
    //    targetTcpPose[6]);

    m_robot->sendCartesianMotionForce(
        targetTcpPose, std::vector<double>(6), m_translationVelLimit, m_rotationVelLimit);

    m_device->setForceAndTorqueAndGripperForce(
        Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), 0);

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

Eigen::Matrix3d MainWindow::RotationAboutXAxis(double angle)
{
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();

  rotation(1,1) = cos(angle);
  rotation(2,1) = sin(angle);
  rotation(1,2) = -sin(angle);
  rotation(2,2) = cos(angle);

  return rotation;
}

Eigen::Matrix3d MainWindow::RotationAboutYAxis(double angle)
{
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();

  rotation(2,2) = cos(angle);
  rotation(0,2) = sin(angle);
  rotation(2,0) = -sin(angle);
  rotation(0,0) = cos(angle);

  return rotation;
}

Eigen::Matrix3d MainWindow::RotationAboutZAxis(double angle)
{
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();

  rotation(0,0) = cos(angle);
  rotation(1,0) = sin(angle);
  rotation(0,1) = -sin(angle);
  rotation(1,1) = cos(angle);

  return rotation;
}

} /* namespace teleoperation */
} /* namespace flexiv */
