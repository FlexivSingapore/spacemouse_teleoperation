#ifndef TELEOPERATION_SPACEMOUSEDEVICE_HPP
#define TELEOPERATION_SPACEMOUSEDEVICE_HPP

#include "flexiv/teleoperation/devices/GenericHapticDevice.hpp"
#include "spnav.h"

struct shm_datastruct
{
    bool m_deviceReady;
    double m_pos[3];
    double m_rot[9];
    double m_force[3];
    double m_torque[3];
};

namespace flexiv {
namespace teleoperation {

class SpaceMouseDevice;
typedef std::shared_ptr<SpaceMouseDevice> SpaceMouseDevicePtr;

class SpaceMouseDevice : public GenericHapticDevice
{
public:
    SpaceMouseDevice(unsigned int a_deviceNumber = 0);
    virtual ~SpaceMouseDevice();

    //! Shared SpaceMouseDevice allocator.
    static SpaceMouseDevicePtr create(unsigned int a_deviceNumber = 0)
    {
        return (std::make_shared<SpaceMouseDevice>(a_deviceNumber));
    }

public:
    //! This method opens a connection to the haptic device.
    virtual bool open();

    //! This method closes the connection to the haptic device.
    virtual bool close();

    //! This method calibrates the haptic device.
    virtual bool calibrate(bool a_forceCalibration = false);

    //! This method returns the position of the device.
    virtual bool getPosition(Eigen::Vector3d& a_position);

    //! This method returns the orientation frame of the device end-effector.
    virtual bool getRotation(Eigen::Matrix3d& a_rotation);

    //! This method returns the gripper angle in radian [rad].
    virtual bool getGripperAngleRad(double& a_angle);

    //! This method returns the status of all user switches [__true__ = __ON__ /
    //! __false__ = __OFF__].
    virtual bool getUserSwitches(unsigned int& a_userSwitches);

    //! This method sends a force [N] and a torque [N*m] and gripper force [N]
    //! to the haptic device.
    virtual bool setForceAndTorqueAndGripperForce(
        const Eigen::Vector3d& a_force, const Eigen::Vector3d& a_torque,
        double a_gripperForce);

public:
    //! This method returns the number of devices available from this class of
    //! device.
    static unsigned int getNumDevices();

    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:
    //! Space mouse device handler
    spnav_event sev;

    //! Integrated position
    Eigen::Vector3d m_pos;

    //! Integrated orientation
    Eigen::Matrix3d m_rot;

    //! Velocity
    Eigen::Vector3d m_vel;

    //! omega x velocity
    Eigen::Vector3d m_omega;

    //! Button status
    bool m_buttons[2];

    //! Time point for keeping track of integration time
    std::chrono::_V2::system_clock::time_point m_tpStart;
};

} // namespace teleoperation
} // namespace flexiv

#endif // TELEOPERATION_SPACEMOUSEDEVICE_HPP
