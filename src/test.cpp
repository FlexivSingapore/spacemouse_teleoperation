#include <iostream>
#include <unistd.h>

#include "flexiv/teleoperation/devices/HapticDeviceHandler.hpp"
#include "flexiv/teleoperation/devices/GenericHapticDevice.hpp"

using namespace flexiv::teleoperation;

int main(int argc, char* argv[])
{
  // a haptic device handler
  HapticDeviceHandler* m_handler;

  // Create a haptic device handler
  m_handler = new HapticDeviceHandler();

  int numDevices = m_handler->getNumDevices();
  printf("Number of available haptic device: %d\n", numDevices);

  int counter = 0;

  if (numDevices > 0) {

    GenericHapticDevicePtr device;

    m_handler->getDevice(device, 0);

    // Initialize the haptic device
    device->open();

    // calibrate device (if necessary)
    device->calibrate(true);

    while (1) {
      // Get the position of the haptic device
      Eigen::Vector3d position;
      device->getPosition(position);

      // Get the orientation of the haptic device
      Eigen::Matrix3d rotation;
      device->getRotation(rotation);

      // Get the linear velocity of the haptic device
      Eigen::Vector3d velLinear;
      device->getLinearVelocity(velLinear);

      // Get the anguar velocity of the haptic device
      Eigen::Vector3d velAngular;
      device->getAngularVelocity(velAngular);

      // Set force and torque and gripper force to zero
      device->setForceAndTorqueAndGripperForce(
          Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), 0);

      counter++;

      if (counter >= 100) {
        counter = 0;

        std::cout << "Position: " << std::endl;
        std::cout << position << std::endl << std::endl;

        std::cout << "Linear velocity: " << std::endl;
        std::cout << velLinear << std::endl << std::endl;

        std::cout << "Orientation: " << std::endl;
        std::cout << rotation << std::endl << std::endl;
      }

      usleep(1000);
    }
  }

  return 0;
}