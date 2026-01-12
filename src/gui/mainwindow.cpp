/**
 * @file mainwindow.cpp
 * @date Oct 11, 2022
 * @author Zhan Fan Quek
 */

#include <spdlog/spdlog.h>

#include "flexiv/teleoperation/gui/mainwindow.hpp"
#include "./ui_mainwindow.h"

namespace {

constexpr double kDefaultTcpOffsetMaxX = 0.2;
constexpr double kDefaultTcpOffsetMinX = -0.2;
constexpr double kDefaultTcpOffsetMaxY = 0.2;
constexpr double kDefaultTcpOffsetMinY = -0.2;
constexpr double kDefaultTcpOffsetMaxZ = 0.2;
constexpr double kDefaultTcpOffsetMinZ = -0.2;

constexpr double kDisplayForceMaxValue = 50;
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
    spdlog::info("No haptic device detected\n");
  } else if (device_count_ != 1) {
    spdlog::info("More than 1 haptic device detected\n");
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
      spdlog::info("Haptic device calibration failed\n");
    }
  }

  // Initialize UI
  InitUI();

  // Set default parameters
  teleoperation_offset_max_.x() = kDefaultTcpOffsetMaxX;
  teleoperation_offset_max_.y() = kDefaultTcpOffsetMaxY;
  teleoperation_offset_max_.z() = kDefaultTcpOffsetMaxZ;
  teleoperation_offset_min_.x() = kDefaultTcpOffsetMinX;
  teleoperation_offset_min_.y() = kDefaultTcpOffsetMinY;
  teleoperation_offset_min_.z() = kDefaultTcpOffsetMinZ;
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
      teleoperation_cmd_vel_.fill(0);
      teleoperation_cmd_acc_.fill(0);
      teleop_cmd_->write(teleoperation_cmd_pose_, teleoperation_cmd_vel_, teleoperation_cmd_acc_);

      // ==== Pre-processing before running teleoperation ====
      // Initial Reference Pose
      //=============================================================================
      // Save current robot TCP position and rotation as the first reference position and rotation
      for (size_t i = 0; i < k_cartPositionDofs; i++) {
        teleoperation_target_pose_.pos[i] = robot_states.tcp_pose[i];
        teleoperation_start_pose_.pos[i] = robot_states.tcp_pose[i];
      }
      Eigen::Quaterniond q(robot_states.tcp_pose[3], robot_states.tcp_pose[4],
          robot_states.tcp_pose[5], robot_states.tcp_pose[6]);
      teleoperation_target_pose_.rot = q.toRotationMatrix();
      teleoperation_start_pose_.rot = q.toRotationMatrix();

      // Set the offset pose to identity
      teleoperation_offset_pose.pos = Eigen::Vector3d::Zero();
      teleoperation_offset_pose.rot = Eigen::Matrix3d::Identity();

      // Cartesian stiffness
      //=============================================================================
      // Set slave Cartesian stiffness
      translational_stiffness_ = ui_->teleop_translational_stiffness_doubleSpinBox->value();
      rotational_stiffness_ = ui_->teleop_rotational_stiffness_doubleSpinBox->value();
      std::array<double, flexiv::tdk::kCartDoF> cartesian_stiffness
          = {translational_stiffness_, translational_stiffness_, translational_stiffness_,
              rotational_stiffness_, rotational_stiffness_, rotational_stiffness_};
      robot_ptr_->SetCartesianImpedance(cartesian_stiffness);

      // Workspace offset
      //=============================================================================
      teleoperation_offset_max_.x() = ui_->max_x_offset_doubleSpinBox->value();
      teleoperation_offset_max_.y() = ui_->max_y_offset_doubleSpinBox->value();
      teleoperation_offset_max_.z() = ui_->max_z_offset_doubleSpinBox->value();

      teleoperation_offset_min_.x() = ui_->min_x_offset_doubleSpinBox->value();
      teleoperation_offset_min_.y() = ui_->min_y_offset_doubleSpinBox->value();
      teleoperation_offset_min_.z() = ui_->min_z_offset_doubleSpinBox->value();

      // Translation and rotation scaling
      //=============================================================================
      translational_scaling_ = ui_->motion_translation_scale_doubleSpinBox->value();
      rotational_scaling_ = ui_->motion_rotation_scale_doubleSpinBox->value();

      // Max contact wrench
      //=============================================================================
      max_contact_force_x_ = ui_->max_tcp_force_x_doubleSpinBox->value();
      max_contact_force_y_ = ui_->max_tcp_force_y_doubleSpinBox->value();
      max_contact_force_z_ = ui_->max_tcp_force_z_doubleSpinBox->value();
      std::array<double, flexiv::tdk::kCartDoF> max_contact_wrench
          = {max_contact_force_x_, max_contact_force_y_, max_contact_force_y_, 40, 40, 40};
      robot_ptr_->SetMaxContactWrench(max_contact_wrench);

      // Setup thread for teleoperation
      thread_teleoperation_
          = std::make_unique<std::thread>(std::bind(&MainWindow::RunTeleoperation, this));

      state_machine_ = SM_TELEOP;
      spdlog::info("Starting teleoperation");
      break;
    }

    case SM_TELEOP: {
      // Update display forces
      ui_->force_lcdNumber->display(display_force_norm_);
      ui_->force_progressBar->setValue(display_force_percentage_);
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
    if (device_count_ > 0) {
      state_machine_ = SM_TELEOP_INIT;
      ui_->set_idle_button->setEnabled(true);
      ui_->set_freedrive_button->setEnabled(false);
      ui_->set_move_home_button->setEnabled(false);
      ui_->set_teleoperation_button->setEnabled(false);
    } else {
      spdlog::error("No haptic device available. Unable to run teleoperation");
    }
    state_machine_ = SM_TELEOP_INIT;
    ui_->set_idle_button->setEnabled(true);
    ui_->set_freedrive_button->setEnabled(false);
    ui_->set_move_home_button->setEnabled(false);
    ui_->set_teleoperation_button->setEnabled(false);
  }
}

void MainWindow::InitUI()
{
  ui_->max_x_offset_doubleSpinBox->setValue(kDefaultTcpOffsetMaxX);
  ui_->max_y_offset_doubleSpinBox->setValue(kDefaultTcpOffsetMaxY);
  ui_->max_z_offset_doubleSpinBox->setValue(kDefaultTcpOffsetMaxZ);
  ui_->min_x_offset_doubleSpinBox->setValue(kDefaultTcpOffsetMinX);
  ui_->min_y_offset_doubleSpinBox->setValue(kDefaultTcpOffsetMinY);
  ui_->min_z_offset_doubleSpinBox->setValue(kDefaultTcpOffsetMinZ);
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
    // ============================
    // Display force value
    // ============================
    // Read current robot state
    auto robot_states = robot_ptr_->states();

    Eigen::Vector3d force_in_base(robot_states.ext_wrench_in_world[0],
        robot_states.ext_wrench_in_world[1], robot_states.ext_wrench_in_world[2]);
    display_force_norm_ = force_in_base.norm();

    double percentage_force_max = force_in_base.norm() / kDisplayForceMaxValue * 100;
    if (percentage_force_max > 100) {
      percentage_force_max = 100;
    }
    display_force_percentage_ = static_cast<int>(percentage_force_max);

    // ==================================
    // Haptic device input
    // ==================================
    // Read input from haptic device
    device_->getPosition(device_pose_.pos);
    device_->getRotation(device_pose_.rot);
    device_->getLinearVelocity(device_velocity_.head(k_cartPositionDofs));
    device_->getAngularVelocity(device_velocity_.tail(k_cartOrientationDofs));

    // Filter device velocity
    for (size_t i = 0; i < k_cartPoseDofs; i++) {
      device_velocity_filtered_[i] = 0.8 * device_velocity_filtered_[i] + 0.2 * device_velocity_[i];
    }

    auto linear_velocity_filtered = device_velocity_filtered_.head(k_cartPositionDofs);
    auto angular_velocity_filtered = device_velocity_filtered_.tail(k_cartOrientationDofs);

    // ============================
    // Limit velocity due to force
    // ============================
    // Get the current robot position
    Eigen::Vector3d robot_current_position(
        robot_states.tcp_pose[0], robot_states.tcp_pose[1], robot_states.tcp_pose[2]);

    // Get the difference between target position and current position
    Eigen::Vector3d robot_target_position(teleoperation_target_pose_.pos.x(),
        teleoperation_target_pose_.pos.y(), teleoperation_target_pose_.pos.z());
    Eigen::Vector3d position_difference = robot_current_position - robot_target_position;

    double position_difference_norm = position_difference.norm();
    double boundary_norm = kDisplayForceMaxValue / translational_stiffness_;

    if (position_difference_norm >= boundary_norm) {
      Eigen::Vector3d position_difference_direction_vector
          = position_difference / position_difference.norm();

      // Only accept linear velocity that is pointing to the robot actual position
      double norm_in_direction_towards_robot
          = linear_velocity_filtered.dot(position_difference_direction_vector);

      if (norm_in_direction_towards_robot > 0) {
        linear_velocity_filtered
            = norm_in_direction_towards_robot * position_difference_direction_vector;
      } else {
        linear_velocity_filtered.setZero();
      }
    }

    // ============================
    // Integrate linear velocity
    // ============================
    // Integrate linear velocity to get new TCP position
    teleoperation_offset_pose.pos.x()
        += linear_velocity_filtered(0) * k_loopPeriod * -1.0 * 0.1 * translational_scaling_;
    teleoperation_offset_pose.pos.y()
        += linear_velocity_filtered(1) * k_loopPeriod * -1.0 * 0.1 * translational_scaling_;
    teleoperation_offset_pose.pos.z()
        += linear_velocity_filtered(2) * k_loopPeriod * 0.1 * translational_scaling_;

    // Limit slave Tcp pose offset to within a bound
    if (teleoperation_offset_pose.pos.x() >= teleoperation_offset_max_.x()) {
      teleoperation_offset_pose.pos.x() = teleoperation_offset_max_.x();
    } else if (teleoperation_offset_pose.pos.x() <= teleoperation_offset_min_.x()) {
      teleoperation_offset_pose.pos.x() = teleoperation_offset_min_.x();
    }

    if (teleoperation_offset_pose.pos.y() >= teleoperation_offset_max_.y()) {
      teleoperation_offset_pose.pos.y() = teleoperation_offset_max_.y();
    } else if (teleoperation_offset_pose.pos.y() <= teleoperation_offset_min_.y()) {
      teleoperation_offset_pose.pos.y() = teleoperation_offset_min_.y();
    }

    if (teleoperation_offset_pose.pos.z() >= teleoperation_offset_max_.z()) {
      teleoperation_offset_pose.pos.z() = teleoperation_offset_max_.z();
    } else if (teleoperation_offset_pose.pos.z() <= teleoperation_offset_min_.z()) {
      teleoperation_offset_pose.pos.z() = teleoperation_offset_min_.z();
    }

    // ============================
    // Integrate angular velocity
    // ============================
    // Integrate angular velocity to get new TCP orientation
    Eigen::AngleAxisd angle_axis;
    double angular_velocity_norm = angular_velocity_filtered.norm();
    if (angular_velocity_norm <= k_epsilon) {
      angle_axis.angle() = 0;
      angle_axis.axis() = {0, 0, 1};
    } else {
      angle_axis.angle() = angular_velocity_norm * k_loopPeriod * 0.1 * rotational_scaling_;
      Eigen::Vector3d axis = angular_velocity_filtered / angular_velocity_norm;
      angle_axis.axis().x() = axis.x() * -1.0;
      angle_axis.axis().y() = axis.y() * -1.0;
      angle_axis.axis().z() = axis.z();
    }
    teleoperation_offset_pose.rot = angle_axis.toRotationMatrix() * teleoperation_offset_pose.rot;

    // Limit slave rotation
    Eigen::AngleAxisd slave_rotation_angle_axis;
    slave_rotation_angle_axis.fromRotationMatrix(teleoperation_offset_pose.rot);

    if (slave_rotation_angle_axis.angle() >= M_PI / 4) {
      slave_rotation_angle_axis.angle() = M_PI / 4;
    }
    teleoperation_offset_pose.rot = slave_rotation_angle_axis.toRotationMatrix();

    // ============================
    // Calculate the target pose
    // ============================
    // Set target TCP pose
    teleoperation_target_pose_.pos = teleoperation_start_pose_.pos + teleoperation_offset_pose.pos;
    teleoperation_target_pose_.rot = teleoperation_offset_pose.rot * teleoperation_start_pose_.rot;

    // ============================
    // Send target tcp pose to robot
    // ============================
    // Form pose vector
    for (size_t i = 0; i < 3; i++) {
      teleoperation_cmd_pose_[i] = teleoperation_target_pose_.pos[i];
    }
    Eigen::Quaterniond targetQuat(teleoperation_target_pose_.rot);
    teleoperation_cmd_pose_[3] = targetQuat.w();
    teleoperation_cmd_pose_[4] = targetQuat.x();
    teleoperation_cmd_pose_[5] = targetQuat.y();
    teleoperation_cmd_pose_[6] = targetQuat.z();

    // ==================================
    // Teleoperation
    // ==================================
    teleop_cmd_->write(teleoperation_cmd_pose_, teleoperation_cmd_vel_, teleoperation_cmd_acc_);

    device_->setForceAndTorqueAndGripperForce(
        Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), 0);

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

} /* namespace teleoperation */
} /* namespace flexiv */
