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
      break;
    }

    case SM_IDLE: {
      break;
    }

    case SM_TELEOP_INIT: {
      break;
    }

    case SM_TELEOP_PREPROCESS: {
      break;
    }

    case SM_TELEOP: {
      break;
    }

    case SM_TELEOP_EXIT: {
      break;
    }

    default: {
      break;
    }
  }
}

} /* namespace teleoperation */
} /* namespace flexiv */
