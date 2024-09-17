#pragma once

#include <cmath>
#include "../constants.hpp"
#include "../globals.hpp"

/**
 * \brief Encapsulates functions and state for operator control.
 *
 * \details Practically uses Singleton design pattern, but classes would have
 * made it more complicated for beginners to understand. Also makes extensive
 * use of USING_15_INCH_ROBOT global constant and preprocessor directives to
 * make switching between robots not require separate branches, which could make
 * fixes and updates to one branch not apply to the other. Finally, it includes
 * tests for practically all of the fundamental functions except the driver
 * profiles and the Run function.
 *
 */
namespace aon::operator_control {

// ============================================================================
//    _  _     _                 ___             _   _
//   | || |___| |_ __  ___ _ _  | __|  _ _ _  __| |_(_)___ _ _  ___
//   | __ / -_) | '_ \/ -_) '_| | _| || | ' \/ _|  _| / _ \ ' \(_-<
//   |_||_\___|_| .__/\___|_|   |_| \_,_|_||_\__|\__|_\___/_||_/__/
//              |_|
// ============================================================================

/**
 * \brief Scales analog joystick input for easier control.
 *
 * \details Fine joystick control can be difficult, specially for tasks like
 *     rotating. After researching the forums I found that teams scale their
 *     joystick inputs using an exponential function of sorts. This makes small
 *     inputs produce a smaller output and bigger inputs increase speed, so fine
 *     movements can be done without as much of a hassle.
 *
 * \param x The controller's user input between -1 and 1
 * \param t Decrease in sensitivity
 *
 * <a href="https://www.desmos.com/calculator/uhjyivyj4r">Demonstration of
 * scaling function in Desmos.</a>
 *
 * \return double
 *
 * \warning Make sure that the input x is between -1 and 1!!!
 */
inline double AnalogInputScaling(const double x, const double t) {
  const double z = 127.0 * x;
  const double a = ::std::exp(-::std::fabs(t) / 10.0);
  const double b = ::std::exp((::std::fabs(z) - 127.0) / 10.0);

  return (a + b * (1 - a)) * z / 127.0;
}

// ============================================================================
//    ___      _
//   |   \ _ _(_)_ _____ _ _ ___
//   | |) | '_| \ V / -_) '_(_-<
//   |___/|_| |_|\_/\___|_| /__/
//
// ============================================================================

/// Enrique's Operator Control configuration
inline void _OpControlEnrique() {
#if USING_15_INCH_ROBOT
#endif
}

/// Manes's Operator Control configuration
inline void _OpControlManes() {
#if USING_15_INCH_ROBOT

  //////////// DRIVE ////////////
  const double vertical = AnalogInputScaling(main_controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0, 20);
  const double turn = AnalogInputScaling(main_controller.get_analog(::pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0, 20);

  driveLeft.moveVelocity(static_cast<int>(driveLeft.getGearing()) * std::clamp(vertical + turn, -1.0, 1.0));
  driveRight.moveVelocity(static_cast<int>(driveLeft.getGearing()) * std::clamp(vertical - turn, -1.0, 1.0));

  //////////// INTAKE ////////////

  if (main_controller.get_digital(DIGITAL_R1)) {
    intake.moveVoltage(12000);
  } else if (main_controller.get_digital(DIGITAL_R2)) {
    intake.moveVoltage(-12000);
  } else {
    intake.moveVoltage(0);
  }

#else
  //////////// DRIVE ////////////
  const double leftInput = AnalogInputScaling(main_controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0, 10);
  const double rightInput = AnalogInputScaling(main_controller.get_analog(::pros::E_CONTROLLER_ANALOG_RIGHT_Y) / 127.0, 10);

  driveLeft.moveVelocity(static_cast<int>(driveLeft.getGearing()) * std::clamp(leftInput, -1.0, 1.0));
  driveRight.moveVelocity(static_cast<int>(driveRight.getGearing()) * std::clamp(rightInput, -1.0, 1.0));

  //////////// INTAKE ////////////

  if (main_controller.get_digital(DIGITAL_R1)) {
    intake.moveVoltage(12000);
  } else if (main_controller.get_digital(DIGITAL_R2)) {
    intake.moveVoltage(-12000);
  } else {
    intake.moveVoltage(0);
  }

#endif
}

/// Default Operator Control configuration
inline void _OpControlDefault() { _OpControlManes(); }

// ============================================================================
//    __  __      _        ___             _   _
//   |  \/  |__ _(_)_ _   | __|  _ _ _  __| |_(_)___ _ _
//   | |\/| / _` | | ' \  | _| || | ' \/ _|  _| / _ \ ' \
//   |_|  |_\__,_|_|_||_| |_| \_,_|_||_\__|\__|_\___/_||_|
//
// ============================================================================

/**
 *\brief Main function for operator control.
 *
 * \details Control configurations for the different drivers are manipulated
 * here.
 *
 * \param driver Who is driving the robot?
 *
 * \see aon::operator_control::Drivers
 *
 */
inline void Run(const Drivers driver) {
  switch (driver) {
    case kEnrique:
      _OpControlEnrique();
      break;

    case kManes:
      _OpControlManes();
      break;

    default:
      _OpControlDefault();
      break;
  }
}

// ============================================================================
//    _____       _
//   |_   _|__ __| |_ ___
//     | |/ -_|_-<  _(_-<
//     |_|\___/__/\__/__/
//
// ============================================================================

/**
 * \brief Tests for the operator_control namespace
 *
 * \details Tests helper methods and input scaling. These tests are pretty
 * manual for now, but hopefully next year we'll have automated tests with a
 * solid framework.
 *
 */

//These tests are for SPIN-UP ROBOT SPECIFICALLY
namespace test {
inline bool TestAnalogInputScaling() {
  // (-1, -1), (0, 0) and (1, 1) MUST be in the curve for all values of `t`
  for (int t = 0; t < 1000; t++) {
    if (AnalogInputScaling(1.0, t) != 1.0) return false;
    if (AnalogInputScaling(0, t) != 0) return false;
    if (AnalogInputScaling(-1.0, t) != -1.0) return false;
  }

  // For t = 0, function must be linear with a slope of 1 and Y intercept of 0
  for (double x = -1.0; x <= 1.0; x += 1.0 / (1 << 10))
    if (AnalogInputScaling(x, 0) != x) return false;

  // Test a couple of values for t = 1, 2, 3, 4, and 5
  const double values[5][16] = {
      // t = 1
      {-1.0, -0.7993598934745575, -0.665907684410978, -0.5432575923824988,
       -0.4223082603458672, -0.3016191456359972, -0.18096821992325748,
       -0.060322539673251165, 0.060322539673251165, 0.18096821992325748,
       0.3016191456359972, 0.4223082603458672, 0.5432575923824988,
       0.665907684410978, 0.7993598934745575, 1.0},
      // t = 2
      {-1.0, -0.7384582066030775, -0.6048984343306752, -0.4919149387807351,
       -0.382171114502365, -0.2729229619248319, -0.16374755317798959,
       -0.05458213618483686, 0.05458213618483686, 0.16374755317798959,
       0.2729229619248319, 0.382171114502365, 0.4919149387807351,
       0.6048984343306752, 0.7384582066030775, 1.0},
      // t = 3
      {-1.0, -0.6833520815002528, -0.5496949820117037, -0.44545818466060066,
       -0.34585352308999784, -0.24695758114813546, -0.14816564954334369,
       -0.04938800431389544, 0.04938800431389544, 0.14816564954334369,
       0.24695758114813546, 0.34585352308999784, 0.44545818466060066,
       0.5496949820117037, 0.6833520815002528, 1.0},
      // t = 4
      {-1.0, -0.6334899975442465, -0.49974483274873444, -0.4034223752122067,
       -0.3129920074471465, -0.22346313304782897, -0.1340665600904855,
       -0.044688159442854515, 0.044688159442854515, 0.1340665600904855,
       0.22346313304782897, 0.3129920074471465, 0.4034223752122067,
       0.49974483274873444, 0.6334899975442465, 1.0},
      // t = 5
      {-1.0, -0.5883729182396015, -0.4545480686591184, -0.3653868019258703,
       -0.28325767848012057, -0.20220447729056773, -0.12130917639330331,
       -0.04043556394457229, 0.04043556394457229, 0.12130917639330331,
       0.20220447729056773, 0.28325767848012057, 0.3653868019258703,
       0.4545480686591184, 0.5883729182396015, 1.0}};

  // Output values are so accurate that != is enough.
  for (int t = 0; t < 5; t++) {
    for (int i = 0; i <= 15; i++) {
      const double x = (2 * i - 15.0) / 15.0;
      if (values[t][i] != AnalogInputScaling(x, t + 1)) return false;
    }
  }

  return true;
}
}  // namespace test
}  // namespace aon::operator_control
