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
}

MainWindow::~MainWindow()
{
  delete m_ui;
}

} /* namespace teleoperation */
} /* namespace flexiv */
