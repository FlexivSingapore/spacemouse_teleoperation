/**
 * @file mainwindow.cpp
 * @date Oct 11, 2022
 * @author Zhan Fan Quek
 */

#include <spdlog/spdlog.h>

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
, ui_(new Ui::MainWindow)
{
  ui_->setupUi(this);

  // Start time for state machine and status update
  connect(&timer_, SIGNAL(timeout()), this, SLOT(update()));
  timer_.start(30);

  // Haptic Device Initialization
  //=============================================================================
  // Check if device is connected
  device_count_ = device_handler_.getNumDevices();
  if (device_count_ == 0) {
    printf("No haptic device detected\n");
  } else if (device_count_ != 1) {
    printf("More than 1 haptic device detected\n");
  }

  // Print information of detected haptic devices
  for (unsigned int i = 0; i < device_count_; i++) {
    device_handler_.getDeviceSpecifications(device_info_, 0);
  }

  // Access the first device
  if (device_count_ > 0) {
    device_handler_.getDevice(device_, 0);

    device_->open();

    if (!device_->calibrate(true)) {
      printf("Haptic device calibration failed\n");
    }
  }
}

MainWindow::~MainWindow()
{
  delete ui_;
}

void MainWindow::update()
{
  // Run state machine
  switch (state_machine_) {

    case SM_DISCONNECT: {

      if (robot_ptr_ != nullptr) {
        if (robot_ptr_->connected()) {
          state_machine_ = SM_IDLE;
        }
      }

      break;
    }

    case SM_IDLE: {
      break;
    }

    case SM_FREEDRIVE_INIT: {

      // Set operation mode to primitive execution mode
      robot_ptr_->SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
      state_machine_ = SM_FREEDRIVE_CALISENSOR;

      break;
    }

    case SM_FREEDRIVE_CALISENSOR: {

      // Execute primitive
      robot_ptr_->ExecutePrimitive("ZeroFTSensor", {
                                                       {"dataCollectionTime", 0.1},
                                                       {"enableStaticCheck", false},
                                                   });
      state_machine_ = SM_FREEDRIVE_WAITCALISENSOR;
      break;
    }

    case SM_FREEDRIVE_WAITCALISENSOR: {
      if (std::get<int>(robot_ptr_->primitive_states()["terminated"])) {
        robot_ptr_->ExecutePrimitive("FloatingJoint",
            {{"floatingJoint", std::vector<double> {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}},
                {"dampingLevel", std::vector<double> {20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                {"responseTorque", std::vector<double> {2.0, 2.0, 2.0, 2.0, 1.0, 1.0, 0.5}}});

        state_machine_ = SM_FREEDRIVE;
      }

      break;
    }

    case SM_FREEDRIVE: {
      break;
    }

    case SM_FREEDRIVE_EXIT: {
      // Set operation mode to Idle
      robot_ptr_->SwitchMode(flexiv::rdk::Mode::IDLE);
      state_machine_ = SM_IDLE;
      break;
    }

    case SM_TELEOP_INIT: {
      // Set operation mode to primitive execution mode
      robot_ptr_->SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
      state_machine_ = SM_TELEOP_CALIFORCESENSOR;

      spdlog::info("Moving to state SM_TELEOP_CALIFORCESENSOR");
      break;
    }

    case SM_TELEOP_CALIFORCESENSOR: {
      // Execute primitive
      robot_ptr_->ExecutePrimitive("ZeroFTSensor", {
                                                       {"dataCollectionTime", 0.1},
                                                       {"enableStaticCheck", false},
                                                   });
      state_machine_ = SM_TELEOP_WAITCALISENSOR;
      spdlog::info("Moving to state SM_TELEOP_WAITCALISENSOR");
      break;
    }

    case SM_TELEOP_WAITCALISENSOR: {
      if (std::get<int>(robot_ptr_->primitive_states()["terminated"])) {
        state_machine_ = SM_TELEOP_PREPROCESS;
        spdlog::info("Moving to state SM_TELEOP_PREPROCESS");
      }

      break;
    }

    case SM_TELEOP_PREPROCESS: {

      device_teleop_ptr_->Init();
      device_teleop_ptr_->Start(teleop_cmd_vector_);

      // Update teleoperation pose command to the current pose
      flexiv::rdk::RobotStates robot_states = robot_ptr_->states();
      teleoperation_cmd_pose_ = robot_states.tcp_pose;
      teleop_cmd_->write(teleoperation_cmd_pose_, teleoperation_cmd_vel_, teleoperation_cmd_acc_);

      // Setup thread to run NRT stream cartesian command
      thread_teleoperation_
          = std::make_unique<std::thread>(std::bind(&MainWindow::RunTeleoperation, this));

      state_machine_ = SM_TELEOP;
      spdlog::info("Starting teleoperation");
      break;
    }

    case SM_TELEOP: {
      break;
    }

    case SM_TELEOP_EXIT: {
      device_teleop_ptr_->Stop();

      // Set operation mode to Idle
      robot_ptr_->SwitchMode(flexiv::rdk::Mode::IDLE);

      state_machine_ = SM_IDLE;
      break;
    }

    case SM_MOVEPOSE_INIT: {
      // Set operation mode to plan execution mode
      robot_ptr_->SwitchMode(flexiv::rdk::Mode::NRT_PLAN_EXECUTION);

      // Robot run free drive
      try {
        robot_ptr_->ExecutePlan(ui_->home_plan_name_lineEdit->text().toStdString());
      } catch (const std::exception& e) {
        spdlog::error(e.what());
      }

      state_machine_ = SM_MOVEPOSE_RUNPLAN_WAITFORFINISH;
      break;
    }

    case SM_MOVEPOSE_RUNPLAN_WAITFORFINISH: {
      if (!robot_ptr_->busy()) {
        state_machine_ = SM_MOVEPOSE_EXIT;
      }
      break;
    }

    case SM_MOVEPOSE_EXIT: {

      ui_->set_idle_button->setEnabled(false);
      ui_->set_freedrive_button->setEnabled(true);
      ui_->set_move_home_button->setEnabled(true);
      ui_->set_teleoperation_button->setEnabled(true);

      // Set operation mode to Idle
      robot_ptr_->SwitchMode(flexiv::rdk::Mode::IDLE);
      state_machine_ = SM_IDLE;
      break;
    }

    default: {
      break;
    }
  }

  // Update the robot state and connection status
  if (robot_ptr_ != nullptr && robot_ptr_->connected()) {
    // Check whether E - stop is engaged if (m_rdkClient->isEstopReleased() == false)
    {
      UpdateRobotStatus(true, false);
    }

    // Check robot status
    if (robot_ptr_->operational()) {
      ui_->rdk_operational_status_label->setText("Ready");
      ui_->rdk_operational_status_label->setStyleSheet(
          QStringLiteral("QLabel{color: rgb(0, 255, 0);}"));
      UpdateRobotStatus(true, true);
    } else if (robot_ptr_->busy()) {
      ui_->rdk_operational_status_label->setText("Busy");
      ui_->rdk_operational_status_label->setStyleSheet(
          QStringLiteral("QLabel{color: rgb(255, 165, 0);}"));
    } else if (robot_ptr_->fault()) {
      ui_->rdk_operational_status_label->setText("Fault");
      ui_->rdk_operational_status_label->setStyleSheet(
          QStringLiteral("QLabel{color: rgb(255, 0, 0);}"));
      state_machine_ = SM_DISCONNECT;
      ui_->robot_mode_groupBox->setEnabled(false);
    } else if (robot_ptr_->recovery()) {
      ui_->rdk_operational_status_label->setText("Recovery");
      ui_->rdk_operational_status_label->setStyleSheet(
          QStringLiteral("QLabel{color: rgb(0, 0, 255);}"));
    } else {
      ui_->rdk_operational_status_label->setText("Not Enabled");
      ui_->rdk_operational_status_label->setStyleSheet(
          QStringLiteral("QLabel{color: rgb(211, 211, 211);}"));
      state_machine_ = SM_DISCONNECT;
      ui_->robot_mode_groupBox->setEnabled(false);
      UpdateRobotStatus(true, false);
    }
  } else {
    UpdateRobotStatus(false, false);
  }
}

void MainWindow::on_connect_rdk_button_clicked()
{
  // Check whether
  auto robot_sn = ui_->robot_sn_lineEdit->text().toStdString();

  if (robot_sn == "") {
    spdlog::error("Robot serial is empty");
  }

  try {
    std::vector<std::string> robots_for_teleoperation;
    robots_for_teleoperation.push_back(robot_sn);
    device_teleop_ptr_ = std::make_shared<flexiv::tdk::DeviceTeleopLan>(robots_for_teleoperation);

    device_teleop_ptr_->Init();

    // Get the robot pointer
    robot_ptr_ = device_teleop_ptr_->instance(0);

    // Initialize teleoperation command
    teleop_cmd_vector_.clear();
    teleop_cmd_ = std::make_shared<flexiv::tdk::MotionControlCmds>();
    teleop_cmd_vector_.push_back(teleop_cmd_);

    ui_->robot_mode_groupBox->setEnabled(true);

  } catch (const std::exception& e) {
    spdlog::error(e.what());
  }
}

void MainWindow::on_set_idle_button_clicked()
{
  if (state_machine_ == SM_FREEDRIVE) {
    state_machine_ = SM_FREEDRIVE_EXIT;

    ui_->set_idle_button->setEnabled(false);
    ui_->set_freedrive_button->setEnabled(true);
    ui_->set_move_home_button->setEnabled(true);
    ui_->set_teleoperation_button->setEnabled(true);
  } else if (state_machine_ == SM_TELEOP) {
    state_machine_ = SM_TELEOP_EXIT;

    ui_->set_idle_button->setEnabled(false);
    ui_->set_freedrive_button->setEnabled(true);
    ui_->set_move_home_button->setEnabled(true);
    ui_->set_teleoperation_button->setEnabled(true);
  }
}

void MainWindow::on_set_move_home_button_clicked()
{
  if (state_machine_ == SM_IDLE) {
    state_machine_ = SM_MOVEPOSE_INIT;

    ui_->set_idle_button->setEnabled(false);
    ui_->set_freedrive_button->setEnabled(false);
    ui_->set_move_home_button->setEnabled(false);
    ui_->set_teleoperation_button->setEnabled(false);
  }
}

void MainWindow::on_set_freedrive_button_clicked()
{
  if (state_machine_ == SM_IDLE) {
    state_machine_ = SM_FREEDRIVE_INIT;

    ui_->set_idle_button->setEnabled(true);
    ui_->set_freedrive_button->setEnabled(false);
    ui_->set_move_home_button->setEnabled(false);
    ui_->set_teleoperation_button->setEnabled(false);
  }
}

void MainWindow::on_set_teleoperation_button_clicked()
{
  if (state_machine_ == SM_IDLE) {
    /*
    if (device_count_ > 0) {
      state_machine_ = SM_TELEOP_INIT;
      ui_->set_idle_button->setEnabled(true);
      ui_->set_freedrive_button->setEnabled(false);
      ui_->set_move_home_button->setEnabled(false);
      ui_->set_teleoperation_button->setEnabled(false);
    } else {
      spdlog::error("No haptic device available. Unable to run teleoperation");
    }
    */
    state_machine_ = SM_TELEOP_INIT;
    ui_->set_idle_button->setEnabled(true);
    ui_->set_freedrive_button->setEnabled(false);
    ui_->set_move_home_button->setEnabled(false);
    ui_->set_teleoperation_button->setEnabled(false);
  }
}

void MainWindow::InitUI()
{
  //
}

void MainWindow::UpdateRobotStatus(bool flag_connected, bool flag_operational)
{
  if (flag_connected) {
    ui_->rdk_connection_status_label->setText("Connected");
    ui_->rdk_connection_status_label->setStyleSheet(
        QStringLiteral("QLabel{color: rgb(0, 255, 0);}"));
    if (flag_operational) {
      ui_->connect_rdk_button->setText("RDK Connected/Enabled");
      ui_->connect_rdk_button->setEnabled(false);
    } else {
      ui_->connect_rdk_button->setText("Enable Robot");
      ui_->connect_rdk_button->setEnabled(true);
    }
  } else {
    ui_->rdk_connection_status_label->setText("Disconnected");
    ui_->rdk_connection_status_label->setStyleSheet(
        QStringLiteral("QLabel{color: rgb(255, 0, 0);}"));
    ui_->connect_rdk_button->setText("Connect");
    ui_->connect_rdk_button->setEnabled(true);

    ui_->robot_mode_groupBox->setEnabled(false);
    ui_->set_idle_button->setEnabled(false);
  }
}

void MainWindow::RunTeleoperation()
{
  while ((state_machine_ == SM_TELEOP)) {
    teleoperation_cmd_pose_[2] += 0.001;
    teleop_cmd_->write(teleoperation_cmd_pose_, teleoperation_cmd_vel_, teleoperation_cmd_acc_);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

} /* namespace teleoperation */
} /* namespace flexiv */
