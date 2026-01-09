#include "flexiv/teleoperation/devices/SpaceMouseDevice.hpp"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <unistd.h>

namespace flexiv {
namespace teleoperation {

SpaceMouseDevice::SpaceMouseDevice(unsigned int a_deviceNumber)
{
  // the connection to your device has not yet been established.
  m_deviceReady = false;

  ////////////////////////////////////////////////////////////////////////////
  /*
      STEP 1:

      Here you should define the specifications of your device.
      These values only need to be estimates. Since haptic devices often perform
      differently depending of their configuration withing their workspace,
      simply use average values.
  */
  ////////////////////////////////////////////////////////////////////////////

  //--------------------------------------------------------------------------
  // NAME:
  //--------------------------------------------------------------------------

  // haptic device model (see file "CGenericHapticDevice.h")
  m_specifications.m_model = HAPTIC_DEVICE_SPACEMOUSE;

  // name of the device manufacturer, research lab, university.
  m_specifications.m_manufacturerName = "3Dconnexion";

  // name of your device
  m_specifications.m_modelName = "Space mouse wireless";

  //--------------------------------------------------------------------------
  // CHARACTERISTICS: (The following values must be positive or equal to zero)
  //--------------------------------------------------------------------------

  // the maximum force [N] the device can produce along the x,y,z axis.
  m_specifications.m_maxLinearForce = 5.0; // [N]

  // the maximum amount of torque your device can provide arround its
  // rotation degrees of freedom.
  m_specifications.m_maxAngularTorque = 0.2; // [N*m]

  // the maximum amount of torque which can be provided by your gripper
  m_specifications.m_maxGripperForce = 3.0; // [N]

  // the radius of the physical workspace of the device (x,y,z axis)
  m_specifications.m_workspaceRadius = 10.2; // [m]

  // the maximum opening angle of the gripper
  m_specifications.m_gripperMaxAngleRad = 30.0 / 180.0 * M_PI;

  ////////////////////////////////////////////////////////////////////////////
  /*
      DAMPING PROPERTIES:

      Start with small values as damping terms can be high;y sensitive to
      the quality of your velocity signal and the spatial resolution of your
      device. Try gradually increasing the values by using example "01-devices"
      and by enabling viscosity with key command "2".
  */
  ////////////////////////////////////////////////////////////////////////////

  // Maximum recommended linear damping factor Kv
  m_specifications.m_maxLinearDamping = 20.0; // [N/(m/s)]

  //! Maximum recommended angular damping factor Kv (if actuated torques are
  //! available)
  m_specifications.m_maxAngularDamping = 0.0; // [N*m/(Rad/s)]

  //! Maximum recommended angular damping factor Kv for the force gripper. (if
  //! actuated gripper is available)
  m_specifications.m_maxGripperAngularDamping = 0.0; // [N*m/(Rad/s)]

  //--------------------------------------------------------------------------
  // CHARACTERISTICS: (The following are of boolean type: (true or false)
  //--------------------------------------------------------------------------

  // does your device provide sensed position (x,y,z axis)?
  m_specifications.m_sensedPosition = true;

  // does your device provide sensed rotations (i.e stylus)?
  m_specifications.m_sensedRotation = true;

  // does your device provide a gripper which can be sensed?
  m_specifications.m_sensedGripper = true;

  // is you device actuated on the translation degrees of freedom?
  m_specifications.m_actuatedPosition = true;

  // is your device actuated on the rotation degrees of freedom?
  m_specifications.m_actuatedRotation = true;

  // is the gripper of your device actuated?
  m_specifications.m_actuatedGripper = true;

  // can the device be used with the left hand?
  m_specifications.m_leftHand = true;

  // can the device be used with the right hand?
  m_specifications.m_rightHand = true;

  ////////////////////////////////////////////////////////////////////////////
  /*
      STEP 2:

      Here, you shall  implement code which tells the application if your
      device is actually connected to your computer and can be accessed.
      In practice this may be consist in checking if your I/O board
      is active or if your drivers are available.

      If your device can be accessed, set:
      m_systemAvailable = true;

      Otherwise set:
      m_systemAvailable = false;

      Your actual code may look like:

      bool result = checkIfMyDeviceIsAvailable()
      m_systemAvailable = result;

      If want to support multiple devices, using the method argument
      a_deviceNumber to know which device to setup
  */
  ////////////////////////////////////////////////////////////////////////////
  m_pos.setZero();
  m_rot.setIdentity();

  // Initialize the space mouse
  if (spnav_open() == -1) {
    fprintf(stderr, "failed to connect to the space navigator daemon\n");
    m_deviceAvailable = false;
  } else {
    m_deviceAvailable = true;
  }

  spnav_close();
}

//==============================================================================
/*!
    Destructor of SpaceMouseDevice.
*/
//==============================================================================
SpaceMouseDevice::~SpaceMouseDevice()
{
  // close connection to device
  if (m_deviceReady) {
    spnav_close();
    close();
  }
}

//==============================================================================
/*!
    This method opens a connection to your device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool SpaceMouseDevice::open()
{
  // check if the system is available
  if (!m_deviceAvailable)
    return (false);

  // if system is already opened then return
  if (m_deviceReady)
    return (false);

  ////////////////////////////////////////////////////////////////////////////
  /*
      STEP 3:

      Here you shall implement to open a connection to your
      device. This may include opening a connection to an interface board
      for instance or a USB port.

      If the connection succeeds, set the variable 'result' to true.
      otherwise, set the variable 'result' to false.

      Verify that your device is calibrated. If your device
      needs calibration then call method calibrate() for which you will
      provide code in STEP 5 further below.
  */
  ////////////////////////////////////////////////////////////////////////////
  bool result = false;

  if (spnav_open() == -1) {
    fprintf(stderr, "failed to connect to the space navigator daemon\n");
    m_deviceReady = false;
    result = false;
  } else {
    result = true;
  }

  // update device status
  if (result) {
    m_deviceReady = true;
    return (true);
  } else {
    m_deviceReady = false;
    return (false);
  }
}

//==============================================================================
/*!
    This method closes the connection to your device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool SpaceMouseDevice::close()
{
  // check if the system has been opened previously
  if (!m_deviceReady)
    return (false);

  ////////////////////////////////////////////////////////////////////////////
  /*
      STEP 4:

      Here you shall implement code that closes the connection to your
      device.

      If the operation fails, simply set the variable 'result' to C_ERROR   .
      If the connection succeeds, set the variable 'result' to C_SUCCESS.
  */
  ////////////////////////////////////////////////////////////////////////////

  bool result = true;
  spnav_close();

  // update status
  m_deviceReady = false;

  return (result);
}

//==============================================================================
/*!
    This method calibrates your device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool SpaceMouseDevice::calibrate(bool a_forceCalibration)
{
  (void)a_forceCalibration;

  // check if the device is read. See step 3.
  if (!m_deviceReady)
    return (false);

  ////////////////////////////////////////////////////////////////////////////
  /*
      STEP 5:

      Here you shall implement code that handles a calibration procedure of the
      device. In practice this may include initializing the registers of the
      encoder counters for instance.

      If the device is already calibrated and  a_forceCalibration == false,
      the method may immediately return without further action.
      If a_forceCalibration == true, then the calibrartion procedure
      shall be executed even if the device has already been calibrated.

      If the calibration procedure succeeds, the method returns C_SUCCESS,
      otherwise return C_ERROR.
  */
  ////////////////////////////////////////////////////////////////////////////

  bool result = true;

  m_pos.setZero();
  m_rot.setIdentity();
  m_tpStart = std::chrono::system_clock::now();

  return (result);
}

//==============================================================================
/*!
    This method returns the number of devices available from this class of
   device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
unsigned int SpaceMouseDevice::getNumDevices()
{
  ////////////////////////////////////////////////////////////////////////////
  /*
      STEP 6:

      Here you shall implement code that returns the number of available
      haptic devices of type "cMyCustomDevice" which are currently connected
      to your computer.

      In practice you will often have either 0 or 1 device. In which case
      the code here below is already implemented for you.

      If you have support more than 1 devices connected at the same time,
      then simply modify the code accordingly so that "numberOfDevices" takes
      the correct value.
  */
  ////////////////////////////////////////////////////////////////////////////

  // *** INSERT YOUR CODE HERE, MODIFY CODE below ACCORDINGLY ***
  int numberOfDevices = 0; // At least set to 1 if a device is available.

  if (spnav_open() == -1) {
    numberOfDevices = 0;
  } else {
    numberOfDevices = 1;
  }

  spnav_close();

  return (numberOfDevices);
}

//==============================================================================
/*!
    This method returns the position of your device. Units are meters [m].

    \param   a_position  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool SpaceMouseDevice::getPosition(Eigen::Vector3d& a_position)
{

  // check if the device is read. See step 3.
  if (!m_deviceReady)
    return (false);

  ////////////////////////////////////////////////////////////////////////////
  /*
      STEP 7:

      Here you shall implement code that reads the position (X,Y,Z) from
      your haptic device. Read the values from your device and modify
      the local variable (x,y,z) accordingly.
      If the operation fails return an C_ERROR, C_SUCCESS otherwise

      Note:
      For consistency, units must be in meters.
      If your device is located in front of you, the x-axis is pointing
      towards you (the operator). The y-axis points towards your right
      hand side and the z-axis points up towards the sky.
  */
  ////////////////////////////////////////////////////////////////////////////

  bool result = true;
  double x, y, z;

  // *** INSERT YOUR CODE HERE, MODIFY CODE below ACCORDINGLY ***
  if (spnav_poll_event(&sev)) {
    if (sev.type == SPNAV_EVENT_MOTION) {
      m_vel[0] = sev.motion.x / 500.0;
      m_vel[1] = sev.motion.z / 500.0;
      m_vel[2] = sev.motion.y / 500.0;
      m_omega[0] = sev.motion.rx / 500.0;
      m_omega[1] = sev.motion.rz / 500.0;
      m_omega[2] = sev.motion.ry / 500.0;
    } else {
      if (sev.button.bnum == 0) {
        if (sev.button.press == 0) {
          m_buttons[0] = false;
        } else {
          m_buttons[0] = true;
        }
      } else if (sev.button.bnum == 1) {
        if (sev.button.press == 0) {
          m_buttons[1] = false;
        } else {
          m_buttons[1] = true;
        }
      }
    }
  }

  auto tp_current = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = tp_current - m_tpStart;
  double time = diff.count();

  if (time > 0.01) {
    // Integrate the velocity to get the position (Assumed time step of 0.01)
    m_pos = m_pos + m_vel * time;

    // Limit the position to the workspace limit
    if (m_pos.x() >= m_specifications.m_workspaceRadius) {
      m_pos.x() = m_specifications.m_workspaceRadius;
    } else if (m_pos.x() <= -m_specifications.m_workspaceRadius) {
      m_pos.x() = -m_specifications.m_workspaceRadius;
    }

    if (m_pos.y() >= m_specifications.m_workspaceRadius) {
      m_pos.y() = m_specifications.m_workspaceRadius;
    } else if (m_pos.y() <= -m_specifications.m_workspaceRadius) {
      m_pos.y() = -m_specifications.m_workspaceRadius;
    }

    if (m_pos.z() >= m_specifications.m_workspaceRadius) {
      m_pos.z() = m_specifications.m_workspaceRadius;
    } else if (m_pos.z() <= -m_specifications.m_workspaceRadius) {
      m_pos.z() = -m_specifications.m_workspaceRadius;
    }

    // Integrate the angular velocity to get the orientation
    Eigen::Vector3d t_axis;
    double t_angle;
    if (m_omega.norm() <= 1e-5) {
      t_axis[0] = 0;
      t_axis[1] = 0;
      t_axis[2] = 1;
      t_angle = 0;
    } else {
      t_axis = m_omega.normalized();
      t_angle = m_omega.norm();
    }
    Eigen::AngleAxisd RotAngleAxis(t_angle * 0.01, t_axis);
    m_rot = RotAngleAxis.toRotationMatrix() * m_rot;

    m_tpStart = std::chrono::system_clock::now();
  }

  x = m_pos.x();
  y = m_pos.y();
  z = m_pos.z();

  // store new position values
  a_position[0] = x;
  a_position[1] = y;
  a_position[2] = z;

  // estimate linear velocity
  m_linearVelocity = m_vel;
  m_angularVelocity = m_omega;

  // exit
  return (result);
}

//==============================================================================
/*!
    This method returns the orientation frame of your device end-effector

    \param   a_rotation  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool SpaceMouseDevice::getRotation(Eigen::Matrix3d& a_rotation)
{
  // check if the device is read. See step 3.
  if (!m_deviceReady)
    return (false);

  ////////////////////////////////////////////////////////////////////////////
  /*
      STEP 8:

      Here you shall implement code which reads the orientation frame from
      your haptic device. The orientation frame is expressed by a 3x3
      rotation matrix. The 1st column of this matrix corresponds to the
      x-axis, the 2nd column to the y-axis and the 3rd column to the z-axis.
      The length of each column vector should be of length 1 and vectors need
      to be orthogonal to each other.

      Note:
      If your device is located in front of you, the x-axis is pointing
      towards you (the operator). The y-axis points towards your right
      hand side and the z-axis points up towards the sky.

      If your device has a stylus, make sure that you set the reference frame
      so that the x-axis corresponds to the axis of the stylus.
  */
  ////////////////////////////////////////////////////////////////////////////

  bool result = true;

  a_rotation = m_rot;

  // exit
  return (result);
}

//==============================================================================
/*!
    This method returns the gripper angle in radian.

    \param   a_angle  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool SpaceMouseDevice::getGripperAngleRad(double& a_angle)
{
  // check if the device is read. See step 3.
  if (!m_deviceReady)
    return (false);

  ////////////////////////////////////////////////////////////////////////////
  /*
      STEP 9:
      Here you may implement code which reads the position angle of your
      gripper. The result must be returned in radian.

      If the operation fails return an error code such as C_ERROR for instance.
  */
  ////////////////////////////////////////////////////////////////////////////

  bool result = true;

  // *** INSERT YOUR CODE HERE, MODIFY CODE below ACCORDINGLY ***

  // return gripper angle in radian
  a_angle = 0.0; // a_angle = getGripperAngleInRadianFromMyDevice();

  // estimate gripper velocity
  estimateGripperVelocity(a_angle);

  // exit
  return (result);
}

//==============================================================================
/*!
    This method sends a force [N] and a torque [N*m] and gripper torque [N*m]
    to your haptic device.

    \param   a_force  Force command.
    \param   a_torque  Torque command.
    \param   a_gripperForce  Gripper force command.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool SpaceMouseDevice::setForceAndTorqueAndGripperForce(
    const Eigen::Vector3d& a_force, const Eigen::Vector3d& a_torque, const double a_gripperForce)
{
  // check if the device is read. See step 3.
  if (!m_deviceReady)
    return (false);

  ////////////////////////////////////////////////////////////////////////////
  /*
      STEP 10:

      Here you may implement code which sends a force (fx,fy,fz),
      torque (tx, ty, tz) and/or gripper force (gf) command to your haptic
     device.

      If your device does not support one of more of the force, torque and
      gripper force capabilities, you can simply ignore them.

      Note:
      For consistency, units must be in Newtons and Newton-meters
      If your device is placed in front of you, the x-axis is pointing
      towards you (the operator). The y-axis points towards your right
      hand side and the z-axis points up towards the sky.

      For instance: if the force = (1,0,0), the device should move towards
      the operator, if the force = (0,0,1), the device should move upwards.
      A torque (1,0,0) would rotate the handle counter clock-wise around the
      x-axis.
  */
  ////////////////////////////////////////////////////////////////////////////

  bool result = true;

  // store new force value.
  m_prevForce = a_force;
  m_prevTorque = a_torque;
  m_prevGripperForce = a_gripperForce;

  // *** INSERT YOUR CODE HERE ***

  // exit
  return (result);
}

//==============================================================================
/*!
    This method returns status of all user switches
    [__true__ = __ON__ / __false__ = __OFF__].

    \param  a_userSwitches  Return the 32-bit binary mask of the device buttons.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool SpaceMouseDevice::getUserSwitches(unsigned int& a_userSwitches)
{
  // check if the device is read. See step 3.
  if (!m_deviceReady)
    return (false);

  ////////////////////////////////////////////////////////////////////////////
  /*
      STEP 11:

      Here you shall implement code that reads the status all user switches
      on your device. For each user switch, set the associated bit on variable
      a_userSwitches. If your device only has one user switch, then set
      a_userSwitches to 1, when the user switch is engaged, and 0 otherwise.
  */
  ////////////////////////////////////////////////////////////////////////////
  a_userSwitches = 0x00;
  if (m_buttons[0]) {
    a_userSwitches = a_userSwitches | 0x01;
  }

  if (m_buttons[1]) {
    a_userSwitches = a_userSwitches | 0x02;
  }

  return (true);
}

} // namespace teleoperation
} // namespace flexiv
