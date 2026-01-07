
#ifndef TELEOPERATION_HAPTICDEVICEHANDLER_HPP
#define TELEOPERATION_HAPTICDEVICEHANDLER_HPP

#include "flexiv/teleoperation/devices/GenericHapticDevice.hpp"

namespace flexiv {
namespace teleoperation {

//------------------------------------------------------------------------------
//! Maximum number of devices that can be connected at the same time.
const unsigned int MAX_HAPTIC_DEVICES = 16;
//------------------------------------------------------------------------------

class HapticDeviceHandler {
  //--------------------------------------------------------------------------
  // CONSTRUCTOR & DESTRUCTOR:
  //--------------------------------------------------------------------------

public:
  HapticDeviceHandler();
  virtual ~HapticDeviceHandler();

public:
  //! This method returns the number of devices currently connected to the
  //! computer.
  unsigned int getNumDevices() { return (m_numDevices); }

  //! This method updates information about the devices that are currently
  //! connected to the computer.
  void update();

  //! This method returns the specifications of the i-th device.
  bool getDeviceSpecifications(HapticDeviceInfo &a_deviceSpecifications,
                               unsigned int a_index = 0);

  //! This method returns a handle to the i-th device, if available.
  bool getDevice(GenericHapticDevicePtr &, unsigned int a_index = 0);

  //--------------------------------------------------------------------------
  // PRIVATE MEMBERS:
  //--------------------------------------------------------------------------

private:
  //! Number of devices.
  unsigned int m_numDevices;

  //! Array of available haptic devices.
  GenericHapticDevicePtr m_devices[MAX_HAPTIC_DEVICES];

  //! A default device with no functionalities.
  GenericHapticDevicePtr m_nullHapticDevice;
};

} // namespace teleoperation
} // namespace flexiv

#endif // TELEOPERATION_HAPTICDEVICEHANDLER_HPP
