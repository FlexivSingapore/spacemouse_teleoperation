#include "flexiv/teleoperation/devices/HapticDeviceHandler.hpp"

#if defined(WIN32) | defined(WIN64)
#include <process.h>
#endif

#include "flexiv/teleoperation/devices/SpaceMouseDevice.hpp"

namespace flexiv {
namespace teleoperation {

HapticDeviceHandler::HapticDeviceHandler()
{
    // clear number of devices
    m_numDevices = 0;

    // create a null haptic device. a pointer to this device is returned
    // if no device is found. this insures that applications which forget
    // to address the case when no device is connected start sending commands
    // to a NULL pointer...
    m_nullHapticDevice = GenericHapticDevice::create();

    // clear device table
    unsigned int i;
    for (i = 0; i < MAX_HAPTIC_DEVICES; i++) {
        m_devices[i] = GenericHapticDevicePtr();
    }

    // search for available haptic devices
    update();
}

HapticDeviceHandler::~HapticDeviceHandler()
{
    // clear current list of devices
    for (unsigned int i = 0; i < MAX_HAPTIC_DEVICES; i++) {
        m_devices[i] = GenericHapticDevicePtr();
    }
}

void HapticDeviceHandler::update()
{
    // temp variables
    int count;
    GenericHapticDevicePtr device;

    // clear current list of devices
    m_numDevices = 0;
    for (unsigned int i = 0; i < MAX_HAPTIC_DEVICES; i++) {
        m_devices[i] = nullptr;
    }

    //--------------------------------------------------------------------------
    // search for SpaceMouse device
    //--------------------------------------------------------------------------
    // check for how many devices are available for this class of devices
    count = SpaceMouseDevice::getNumDevices();

    // open all remaining devices
    for (int i = 0; i < count; i++) {
        device = SpaceMouseDevice::create(i);
        m_devices[m_numDevices] = device;
        m_numDevices++;
    }
}

bool HapticDeviceHandler::getDeviceSpecifications(
    HapticDeviceInfo& a_deviceSpecifications, unsigned int a_index)
{
    if (a_index < m_numDevices) {
        a_deviceSpecifications = m_devices[a_index]->getSpecifications();
        return (true);
    } else {
        return (false);
    }
}

bool HapticDeviceHandler::getDevice(
    GenericHapticDevicePtr& a_hapticDevice, unsigned int a_index)
{
    if (a_index < m_numDevices) {
        a_hapticDevice = m_devices[a_index];
        return (true);
    } else {
        a_hapticDevice = m_nullHapticDevice;
        return (false);
    }
}

} // namespace teleoperation
} // namespace flexiv
