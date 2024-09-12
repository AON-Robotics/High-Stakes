#pragma once

#include <string>
#include "../../pros/apix.h"

namespace aon::port {
/// "Moving" v5_device_e to the aon::port namespace
typedef pros::c::v5_device_e v5_device;

v5_device registry[21];
//////////////////////// FUNCTION PROTOTYPES START HERE ////////////////////////

/**
 * \brief Initial configuration of ports and other preparations
 *
 * \details Any initial setup regarding registering ports and such go here.
 */
void Initialize();

/**
 * \brief Registers a device in the given smart port
 *
 * \details Registers a device of the given type in the given port into the
 * registry, if that type of device is detected to be plugged in to that port.
 *
 * \param port The port number to register the device
 * \param device The type of device to register
 *
 * \note The port number is the same as the labelled ones in the V5 Brain
 *
 * \return true For successful binding
 * \return false For errors during binding
 *
 */

bool Bind(std::uint8_t port, v5_device device);

/**
 * \brief Deregisters a devices from the given port
 *
 * \details Removes the device registed in the given port, if there is one.
 *
 * \param port The port number to deregister
 *
 * \note The port number is the same as the labelled ones in the V5 Brain
 *
 * \return true For successful unbind
 * \return false For errors during unbind
 */
bool Unbind(std::uint8_t port);

/**
 * \brief "Decodes" a device type to a human-friendly device name
 *
 * \param device Device enum we want to find the name for
 *
 * \note The port number is the same as the labelled ones in the V5 Brain
 *
 * \return std::string Human-friendly name for the type of device
 */
std::string v5device2string(v5_device device);

/**
 * \brief Returns the type of the device plugged into the port
 *
 * \param port Port number where we want to know what device is connected
 *
 * \note The port number is the same as the labelled ones in the V5 Brain
 *
 * \return Device type that is plugged into the port (NOT what is registered)
 */
v5_device GetPluggedType(std::uint8_t port);

/**
 * \brief Returns the type of the device registered in the port
 *
 * \param port Port number where we want to know what device is registered
 *
 * \note The port number is the same as the labelled ones in the V5 Brain
 *
 * \return Device type that is registered into the port (NOT what is plugged in)
 */
v5_device GetBoundType(std::uint8_t port);

/**
 * \brief Checks if the plugged type matches the registerred type
 *
 * \param port Port number that we want to inspect
 *
 * \note The port number is the same as the labelled ones in the V5 Brain
 *
 * \return true For correct device plugged in to the port
 * \return false For incorrect device plugged in to the port
 */
bool CheckPort(std::uint8_t port);

//////////////////////// FUNCTION DEFINTIONS START HERE ////////////////////////
void Initialize() {
#if USING_15_INCH_ROBOT
  Bind(1, pros::c::E_DEVICE_ROTATION);
  Bind(2, pros::c::E_DEVICE_MOTOR);
  Bind(3, pros::c::E_DEVICE_MOTOR);
  Bind(4, pros::c::E_DEVICE_RADIO);
  Bind(7, pros::c::E_DEVICE_VISION);
  Bind(8, pros::c::E_DEVICE_MOTOR);
  Bind(9, pros::c::E_DEVICE_ROTATION);
  Bind(10, pros::c::E_DEVICE_ROTATION);
  Bind(11, pros::c::E_DEVICE_MOTOR);
  Bind(13, pros::c::E_DEVICE_MOTOR);
  Bind(14, pros::c::E_DEVICE_MOTOR);
  Bind(15, pros::c::E_DEVICE_MOTOR);
  Bind(16, pros::c::E_DEVICE_MOTOR);
  Bind(17, pros::c::E_DEVICE_MOTOR);
  Bind(18, pros::c::E_DEVICE_MOTOR);
  Bind(19, pros::c::E_DEVICE_MOTOR);
  Bind(20, pros::c::E_DEVICE_MOTOR);

#else
  Bind(1, pros::c::E_DEVICE_MOTOR);
  Bind(2, pros::c::E_DEVICE_ROTATION);
  Bind(6, pros::c::E_DEVICE_MOTOR);
  Bind(8, pros::c::E_DEVICE_ROTATION);
  Bind(9, pros::c::E_DEVICE_MOTOR);
  Bind(10, pros::c::E_DEVICE_MOTOR);
  Bind(11, pros::c::E_DEVICE_MOTOR);
  Bind(12, pros::c::E_DEVICE_MOTOR);
  Bind(13, pros::c::E_DEVICE_MOTOR);
  Bind(14, pros::c::E_DEVICE_MOTOR);
  Bind(15, pros::c::E_DEVICE_IMU);
  Bind(16, pros::c::E_DEVICE_ROTATION);
  Bind(17, pros::c::E_DEVICE_MOTOR);
  Bind(18, pros::c::E_DEVICE_MOTOR);
  Bind(19, pros::c::E_DEVICE_MOTOR);
  Bind(20, pros::c::E_DEVICE_MOTOR);
  Bind(21, pros::c::E_DEVICE_RADIO);
#endif
}

void Run() {
  for (int i = 1; i <= 21; i += 1) {
    bool portinfo = CheckPort(i);
    if (portinfo == true) {
      std::cout << "Port " << i << " is connected correctly \n";

    } else
      std::cout << "Port " << i
                << " is INCORRECTLY connected. It currently has `"
                << v5device2string(GetPluggedType(i))
                << "` but it should have `" << v5device2string(GetBoundType(i))
                << "`\n";
  }
}
bool Bind(std::uint8_t port, v5_device device) {
  registry[port - 1] = device;
  return true;
}
bool Unbind(std::uint8_t port) {
  registry[port - 1] = pros::c::E_DEVICE_NONE;
  return true;
}

std::string v5device2string(v5_device device) {
  switch (device) {
    case pros::c::E_DEVICE_NONE:
      return "none";
    case pros::c::E_DEVICE_ADI:
      return "adi";
    case pros::c::E_DEVICE_DISTANCE:
      return "distance";
    case pros::c::E_DEVICE_GPS:
      return "gps";
    case pros::c::E_DEVICE_IMU:
      return "imu";
    case pros::c::E_DEVICE_MOTOR:
      return "motor";
    case pros::c::E_DEVICE_OPTICAL:
      return "optical";
    case pros::c::E_DEVICE_RADIO:
      return "radio";
    case pros::c::E_DEVICE_ROTATION:
      return "rotation";
    case pros::c::E_DEVICE_SERIAL:
      return "serial";
    case pros::c::E_DEVICE_UNDEFINED:
      return "undefined";
    case pros::c::E_DEVICE_VISION:
      return "vision";
      break;
  }
  return "error";
}

v5_device GetPluggedType(std::uint8_t port) {
  return pros::c::registry_get_plugged_type(port - 1);
}

v5_device GetBoundType(std::uint8_t port) { return registry[port - 1]; }

bool CheckPort(std::uint8_t port) {
  if (GetPluggedType(port) != GetBoundType(port))
    return false;
  else
    return true;
}

}  // namespace aon::port