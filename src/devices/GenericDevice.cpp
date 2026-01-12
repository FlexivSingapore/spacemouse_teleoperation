#include "flexiv/teleoperation/devices/GenericDevice.hpp"

namespace flexiv {
namespace teleoperation {

GenericDevice::GenericDevice(unsigned int a_deviceNumber)
{
  (void)a_deviceNumber;

  //* Flag indicating availability of device */
  m_deviceAvailable = false;

  //* Flag indicating whether device is ready to receive commands */
  m_deviceReady = false;

  //* Device ID */
  m_deviceNumber = -1;
};

} // namespace teleoperation
} // namespace flexiv
