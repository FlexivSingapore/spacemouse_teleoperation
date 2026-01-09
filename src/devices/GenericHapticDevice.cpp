#include "flexiv/teleoperation/devices/GenericHapticDevice.hpp"

namespace flexiv {
namespace teleoperation {

GenericHapticDevice::GenericHapticDevice(unsigned int a_deviceNumber)
: GenericDevice(a_deviceNumber)
{

  m_tpStart = std::chrono::system_clock::now();

  // set default values
  m_specifications.m_manufacturerName = "FLEXIV";
  m_specifications.m_modelName = "no device";
  m_specifications.m_maxLinearForce = 0.1;           // [N]
  m_specifications.m_maxAngularTorque = 0.1;         // [N*m]
  m_specifications.m_maxGripperForce = 0.1;          // [N]
  m_specifications.m_maxLinearDamping = 0.0;         // [N/(m/s)]
  m_specifications.m_maxAngularDamping = 0.0;        // [N*m/(Rad/s)]
  m_specifications.m_maxGripperAngularDamping = 0.0; // [N*m/(Rad/s)]
  m_specifications.m_workspaceRadius = 0.1;          // [m]
  m_specifications.m_gripperMaxAngleRad = 0.0;
  m_specifications.m_sensedPosition = false;
  m_specifications.m_sensedRotation = false;
  m_specifications.m_sensedGripper = false;
  m_specifications.m_actuatedPosition = false;
  m_specifications.m_actuatedRotation = false;
  m_specifications.m_actuatedGripper = false;
  m_specifications.m_leftHand = false;
  m_specifications.m_rightHand = false;

  // initialize variables about the state of the haptic device
  m_prevForce.setZero();
  m_prevTorque.setZero();
  m_prevGripperForce = 0.0;

  m_angularVelocity.setZero();
  m_linearVelocity.setZero();
  m_gripperAngularVelocity = 0.0;

  // reset history tables for velocity estimation
  auto tp_current = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = tp_current - m_tpStart;
  double time = diff.count();

  m_indexHistoryPos = 0;
  m_indexHistoryPosWin = DEVICE_HISTORY_SIZE - 1;
  for (int i = 0; i < DEVICE_HISTORY_SIZE; i++) {
    m_historyPos[i].m_pos.setZero();
    m_historyPos[i].m_time = time;
  }

  m_indexHistoryRot = 0;
  m_indexHistoryRotWin = DEVICE_HISTORY_SIZE - 1;
  for (int i = 0; i < DEVICE_HISTORY_SIZE; i++) {
    m_historyRot[i].m_rot.setIdentity();
    m_historyRot[i].m_time = time;
  }

  m_indexHistoryGripper = 0;
  m_indexHistoryGripperWin = DEVICE_HISTORY_SIZE - 1;
  for (int i = 0; i < DEVICE_HISTORY_SIZE; i++) {
    m_historyGripper[i].m_value = 0.0;
    m_historyGripper[i].m_time = time;
  }

  // set window time interval for measuring linear velocity
  m_linearVelocityWindowSize = 0.015; // [s]

  // set window time interval for measuring angular velocity
  m_angularVelocityWindowSize = 0.030; // [s]

  // set window time interval for measuring gripper linear velocity
  m_gripperVelocityWindowSize = 0.015; // [s]

  // set gripper settings to emulate user switch
  m_gripperUserSwitchEnabled = false;
  m_gripperUserSwitchAngleStart = 10.0 / 180.0 * M_PI;
  m_gripperUserSwitchAngleClick = 5.0 / 180.0 * M_PI;
  m_gripperUserSwitchForceClick = 3;
  m_gripperUserSwitchForceEngaged = 2;

  // set settings to emulate gripper with user switch
  m_virtualGripperAngleMin = 5.0 / 180.0 * M_PI;
  m_virtualGripperAngleMax = 25.0 / 180.0 * M_PI;
  m_virtualGripperAngularVelocity = 80.0 / 180.0 * M_PI;
  m_virtualGripperAngle = m_virtualGripperAngleMin;
  m_tpVirtualGripperStart = std::chrono::system_clock::now();
}

bool GenericHapticDevice::getUserSwitch(int a_switchIndex, bool& a_status)
{
  // get status of all user switches
  unsigned int userSwitches;
  bool result = getUserSwitches(userSwitches);

  // check for errors
  if (result == false) {
    a_status = false;
    return (false);
  }

  // check particular bit
  if ((userSwitches & (1 << a_switchIndex)) > 0) {
    a_status = true;
  } else {
    a_status = false;
  }

  // success
  return (true);
}

bool GenericHapticDevice::getTransform(Eigen::Matrix4d& a_transform)
{
  Eigen::Vector3d pos(0, 0, 0);
  Eigen::Matrix3d rot;
  rot.setIdentity();

  bool result0 = getPosition(pos);
  bool result1 = getRotation(rot);

  a_transform.setIdentity();
  a_transform.block(0, 0, 3, 3) = rot;
  a_transform.block(0, 3, 3, 1) = pos;

  bool result = result0 | result1;
  return (result);
}

void GenericHapticDevice::estimateLinearVelocity(Eigen::Vector3d& a_newPosition)
{
  // get current time
  auto tp_current = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = tp_current - m_tpStart;
  double time = diff.count();

  // check the time interval between the current and previous sample
  if ((time - m_historyPos[m_indexHistoryPos].m_time) < DEVICE_MIN_ACQUISITION_TIME) {
    return;
  }

  // store new value
  m_indexHistoryPos = (m_indexHistoryPos + 1) % DEVICE_HISTORY_SIZE;
  m_historyPos[m_indexHistoryPos].m_time = time;
  m_historyPos[m_indexHistoryPos].m_pos = a_newPosition;

  // search table to find a sample that occurred before current time
  // minus time window interval
  int i = 0;
  bool completed = false;
  while ((i < DEVICE_HISTORY_SIZE) && (!completed)) {
    double interval = time - m_historyPos[m_indexHistoryPosWin].m_time;
    if ((interval < m_linearVelocityWindowSize) || (i == (DEVICE_HISTORY_SIZE - 1))) {
      // compute result
      Eigen::Vector3d result;
      result = m_historyPos[m_indexHistoryPos].m_pos - m_historyPos[m_indexHistoryPosWin].m_pos;
      if (interval > 0) {
        m_linearVelocity = result / interval;
        completed = true;
      } else {
        completed = true;
      }
    } else {
      m_indexHistoryPosWin = (m_indexHistoryPosWin + 1) % DEVICE_HISTORY_SIZE;
    }
  }
}

void GenericHapticDevice::estimateAngularVelocity(Eigen::Matrix3d& a_newRotation)
{
  // get current time
  auto tp_current = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = tp_current - m_tpStart;
  double time = diff.count();

  // check the time interval between the current and previous sample
  if ((time - m_historyRot[m_indexHistoryRot].m_time) < DEVICE_MIN_ACQUISITION_TIME) {
    return;
  }

  // store new value
  m_indexHistoryRot = (m_indexHistoryRot + 1) % DEVICE_HISTORY_SIZE;
  m_historyRot[m_indexHistoryRot].m_time = time;
  m_historyRot[m_indexHistoryRot].m_rot = a_newRotation;

  // search table to find a sample that occurred before current time
  // minus time window interval
  int i = 0;
  bool completed = false;
  while ((i < DEVICE_HISTORY_SIZE) && (!completed)) {
    double interval = time - m_historyRot[m_indexHistoryRotWin].m_time;
    if ((interval < m_angularVelocityWindowSize) || (i == (DEVICE_HISTORY_SIZE - 1))) {
      // compute result
      if (interval > 0) {
        Eigen::Matrix3d mat = m_historyRot[m_indexHistoryRotWin].m_rot.transpose()
                              * m_historyRot[m_indexHistoryRot].m_rot;
        Eigen::AngleAxisd angleAxis;
        angleAxis.fromRotationMatrix(mat);

        double angle = angleAxis.angle();
        Eigen::Vector3d axis = angleAxis.axis();

        angle = angle / interval;
        m_angularVelocity = a_newRotation * (angle * axis);

        completed = true;
      } else {
        completed = true;
      }
    } else {
      m_indexHistoryRotWin = (m_indexHistoryRotWin + 1) % DEVICE_HISTORY_SIZE;
    }
  }
}

void GenericHapticDevice::estimateGripperVelocity(double a_newGripperAngle)
{
  // get current time
  auto tp_current = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = tp_current - m_tpStart;
  double time = diff.count();

  // check the time interval between the current and previous sample
  if ((time - m_historyGripper[m_indexHistoryGripper].m_time) < DEVICE_MIN_ACQUISITION_TIME) {
    return;
  }

  // store new value
  m_indexHistoryGripper = (m_indexHistoryGripper + 1) % DEVICE_HISTORY_SIZE;
  m_historyGripper[m_indexHistoryGripper].m_time = time;
  m_historyGripper[m_indexHistoryGripper].m_value = a_newGripperAngle;

  // search table to find a sample that occurred before current time
  // minus time window interval
  int i = 0;
  bool completed = false;
  while ((i < DEVICE_HISTORY_SIZE) && (!completed)) {
    double interval = time - m_historyGripper[m_indexHistoryGripperWin].m_time;
    if ((interval < m_gripperVelocityWindowSize) || (i == (DEVICE_HISTORY_SIZE - 1))) {
      // compute result
      if (interval > 0) {
        m_gripperAngularVelocity = (m_historyGripper[m_indexHistoryGripper].m_value
                                       - m_historyGripper[m_indexHistoryGripperWin].m_value)
                                   / interval;
        completed = true;
      } else {
        completed = true;
      }
    } else {
      m_indexHistoryGripperWin = (m_indexHistoryGripperWin + 1) % DEVICE_HISTORY_SIZE;
    }
  }
}

double GenericHapticDevice::computeGripperUserSwitchForce(
    const double& a_gripperAngle, const double& a_gripperAngularVelocity)
{
  (void)a_gripperAngularVelocity;

  if (m_gripperUserSwitchEnabled) {
    // compute damping term
    double damping = 0.0;
    double gripperAngularVelocity = 0.0;
    getGripperAngularVelocity(gripperAngularVelocity);
    damping = -0.1 * m_specifications.m_maxGripperAngularDamping * gripperAngularVelocity;

    // PHASE 0: outside of switch, zero force
    if (a_gripperAngle > m_gripperUserSwitchAngleStart) {
      double force = 0.0;
      return (force);
    }

    // PHASE 1: switch is being engaged. (Force is rising until "click")
    else if ((a_gripperAngle <= m_gripperUserSwitchAngleStart)
             && (a_gripperAngle > m_gripperUserSwitchAngleClick)) {
      double force = (m_gripperUserSwitchAngleStart - a_gripperAngle)
                     * ((m_gripperUserSwitchForceClick)
                         / (m_gripperUserSwitchAngleStart - m_gripperUserSwitchAngleClick));
      return (force + damping);
    }

    // PHASE 2: switch has been engaged. (Force is constant)
    else if (a_gripperAngle <= m_gripperUserSwitchAngleClick) {
      double force = m_gripperUserSwitchForceEngaged;
      return (force + damping);
    }
  }

  return (0.0);
}

bool GenericHapticDevice::getGripperUserSwitch()
{
  if (m_gripperUserSwitchEnabled && m_specifications.m_sensedGripper
      && m_specifications.m_actuatedGripper) {
    double gripperAngle;
    if (getGripperAngleDeg(gripperAngle)) {
      if (gripperAngle < m_gripperUserSwitchAngleClick) {
        return (true);
      } else {
        return (false);
      }
    } else {
      return (false);
    }
  } else {
    return (false);
  }
}

bool GenericHapticDevice::getGripperAngleRad(double& a_angle)
{
  // get user switch
  bool userSwitch = false;
  getUserSwitch(0, userSwitch);

  // read clock time
  auto tpVirtualGripperCurrent = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = tpVirtualGripperCurrent - m_tpVirtualGripperStart;
  double timeElapsed = diff.count();
  m_tpVirtualGripperStart = tpVirtualGripperCurrent;

  // update position
  double nextAngle;
  if (userSwitch) {
    // simulating the closing of the virtual gripper
    nextAngle = m_virtualGripperAngle - m_virtualGripperAngularVelocity * timeElapsed;
    m_gripperAngularVelocity = -m_virtualGripperAngularVelocity;
  } else {
    // simulating the opening of the virtual gripper
    nextAngle = m_virtualGripperAngle + m_virtualGripperAngularVelocity * timeElapsed;
    m_gripperAngularVelocity = m_virtualGripperAngularVelocity;
  }

  // clamp and update new virtual gripper position
  if (nextAngle >= m_virtualGripperAngleMax) {
    m_virtualGripperAngle = m_virtualGripperAngleMax;
  } else if (nextAngle <= m_virtualGripperAngleMin) {
    m_virtualGripperAngle = m_virtualGripperAngleMin;
  } else {
    m_virtualGripperAngle = nextAngle;
  }

  // return value
  a_angle = m_virtualGripperAngle;

  // return success
  return (true);
}

} // namespace teleoperation
} // namespace flexiv
