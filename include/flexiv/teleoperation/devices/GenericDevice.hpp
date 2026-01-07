#ifndef FLEXIV_TELEOPERATION_GENERICDEVICE_HPP
#define FLEXIV_TELEOPERATION_GENERICDEVICE_HPP

const unsigned int MAX_DEVICES = 16;

namespace flexiv {
namespace teleoperation {

/**
 * @class GenericDevice
 * @brief Abstract class for hardware devices
 */
class GenericDevice
{
public:
    GenericDevice(unsigned int a_deviceNumber = 0);
    virtual ~GenericDevice(){};

public:
    virtual bool open() { return false; }
    virtual bool close() { return false; }

    //! This method returns __true__ if the device is available for
    //! communication,
    //! __false__ otherwise.
    bool isDeviceAvailable() { return m_deviceAvailable; }

    //! This method returns __true__ if the connection to the device has been
    //! established by calling method open(), __false__ otherwise.
    bool isDeviceReady() { return m_deviceReady; }

public:
    //! This method returns the number of haptic devices available for this
    //! class of devices.
    static unsigned int getNumDevices() { return (0); }

protected:
    //* Flag that indicates if the device is available to the computer. */
    bool m_deviceAvailable;

    //* Flag that indicates if connection to device was opened successfully by
    // calling method open(). */
    bool m_deviceReady;

    //* Device number ID for this category of devices. Value must be equal or
    // bigger than __0__. A value of __-1__ means that the ID has not yet been
    // defined. */
    int m_deviceNumber;

protected:
    static bool openLibraries() { return (true); }
    static bool closeLibraries() { return (true); }
};

} // namespace teleoperation
} // namespace flexiv

#endif // FLEXIV_TELEOPERATION_GENERICDEVICE_HPP
