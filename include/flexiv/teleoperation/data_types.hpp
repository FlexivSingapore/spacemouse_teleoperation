/**
 * @file config.h
 * @copyright Flexiv Robotics 2019
 */

#ifndef FLEXIV_TELEOPERATION_DATA_TYPES_H_
#define FLEXIV_TELEOPERATION_DATA_TYPES_H_

#include <string>
#include <mutex>
#include <eigen3/Eigen/Eigen>

// clang-format off
namespace flexiv {
namespace teleoperation {
  // Loop period [second]
  constexpr double k_loopPeriod = 0.001;

  // Small number
  constexpr double k_epsilon = 1e-8;

  // Cartesian pose dofs
  constexpr unsigned int k_cartPoseDofs = 6;

  // Cartesian position dofs
  constexpr unsigned int k_cartPositionDofs = 3;

  // Cartesian orientation dofs
  constexpr unsigned int k_cartOrientationDofs = k_cartPoseDofs - k_cartPositionDofs;

  typedef Eigen::Matrix<int, k_cartPoseDofs, 1> Vec6i;
  typedef Eigen::Matrix<double, k_cartPoseDofs, 1> Vec6d;
  typedef Eigen::Matrix<double, k_cartPoseDofs, k_cartPoseDofs> Mat6d;

  struct Pose
  {
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
  };

  struct MasterParameters
  {
    double forceFeedbackGain = 0.0;
    double forceDamping = 0.0;
    double momentDamping = 0.0;

    Vec6i virtualConstraints = Vec6i::Zero();
    bool flagTCPFrame = false;
    bool flagRefreshVirtualConstraint = false;

    double tcpXMin = -0.4;
    double tcpYMin = -0.2;
    double tcpZMin = -0.2;

    double tcpXMax = 0.1;
    double tcpYMax = 0.2;
    double tcpZMax = 0.2;

    mutable std::mutex mutex;
  };

  struct SlaveStatus
  {
    // Robot joint angles
    std::vector<double> q;

    mutable std::mutex mutex;
  };

  struct SharedData
  {
    MasterParameters masterParameters;

    SlaveStatus slaveStatus;
  };


}
}

// clang-format on

#endif
