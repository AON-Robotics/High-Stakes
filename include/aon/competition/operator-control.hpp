#ifndef AON_COMPETITION_OPERATOR_CONTROL_HPP_
#define AON_COMPETITION_OPERATOR_CONTROL_HPP_

#include <cmath>
#include <float.h>
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
//    ___         _       _
//   | _ \_ _ ___| |_ ___| |_ _  _ _ __  ___ ___
//   |  _/ '_/ _ \  _/ _ \  _| || | '_ \/ -_|_-<
//   |_| |_| \___/\__\___/\__|\_, | .__/\___/__/
//                            |__/|_|
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
 * \return Processed value with limits of -1 to 1
 *
 * \warning Make sure that the input x is between -1 and 1!!!
 */
inline double AnalogInputScaling(const double x, const double t);

/**
 * \brief Creates deadzone around `c` percent of user input.
 *
 * \details Moving forward in a straight line might be difficult if the sideways
 * motion (strafing) is also in the same joystick. Creating a deadzone might
 * help the driver have straighter lines while still having decent control over
 * the strafing motion. Note that creating a deadzone of `c` percent also
 * reduces the maximum output value by `c` percent.
 *
 * \param x The controller's user input between -1 and 1
 * \param c Deadzone size
 *
 * <a href="https://www.desmos.com/calculator/vjoehfxgw4">Demonstration of
 * deadzone function in Desmos.</a>
 *
 * \return Processed value with limits of -1 to 1
 *
 * \warning Make sure that the input x is between -1 and 1!!!
 */
inline double Deadzone(const double x, const double c);

/**
 * \brief Applies the helper functions to the driver joystick inputs
 *
 * \details Applies the AnalogInputScaling and the Deadzone helper functions to
 * the desired joystick measurement. The variable will be manipulated to end up
 * between -1 and 1
 *
 * \param x Motion magnitude from -127 to 127
 * \param t AnalogInputScaling sensitivity
 * \param c Deadzone deadzone size
 *
 * \return Preprocessed driver joystick input
 */
inline double PreprocessJoysticks(double x, const double t = DBL_EPSILON,
                                  const double c = 0.0);

/**
 * \brief Operator Control configuration for Enrique Báez
 */
inline void _OpControlEnrique();

/**
 * \brief Operator Control configuration for Gabriel Manes
 */
inline void _OpControlManes();

/**
 * \brief Default Operator Control configuration
 */
inline void _OpControlDefault();

/**
 *\brief Main function for operator control.
 *
 * \details Selection of operator control driver is done here.
 *
 * \param driver Who is driving the robot?
 *
 * \see aon::operator_control::Drivers
 *
 */
inline void Run(const Drivers driver = kDefault);
};  // namespace aon::operator_control

// ============================================================================
//    _  _     _                 ___             _   _
//   | || |___| |_ __  ___ _ _  | __|  _ _ _  __| |_(_)___ _ _  ___
//   | __ / -_) | '_ \/ -_) '_| | _| || | ' \/ _|  _| / _ \ ' \(_-<
//   |_||_\___|_| .__/\___|_|   |_| \_,_|_||_\__|\__|_\___/_||_/__/
//              |_|
// ============================================================================
inline double aon::operator_control::AnalogInputScaling(const double x,
                                                        const double t) {
  const double z = 127.0 * x;
  const double a = ::std::exp(-::std::fabs(t) / 10.0);
  const double b = ::std::exp((::std::fabs(z) - 127.0) / 10.0);

  return (a + b * (1 - a)) * z / 127.0;
}

inline double aon::operator_control::Deadzone(const double x, const double c) {
  const double sign = (x > 0.0) ? 1.0 : (x < 0.0) ? -1.0 : 0.0;
  const double clamped_c = std::clamp(c, 0.0, 1.0);
  return sign * std::max(0.0, std::fabs(x) - clamped_c);
}

inline double aon::operator_control::PreprocessJoysticks(double x,
                                                         const double t,
                                                         const double c) {
  x /= 127.0;
  x = AnalogInputScaling(x, t);
  x = Deadzone(x, c);

  return x;
}

// ============================================================================
//    ___      _                 ___         _           _
//   |   \ _ _(_)_ _____ _ _    / __|___ _ _| |_ _ _ ___| |___
//   | |) | '_| \ V / -_) '_|  | (__/ _ \ ' \  _| '_/ _ \ (_-<
//   |___/|_| |_|\_/\___|_|     \___\___/_||_\__|_| \___/_/__/
//
// ============================================================================



inline void aon::operator_control::_OpControlEnrique() {

// #if USING_15_INCH_ROBOT

// #else

// #endif

}

inline void aon::operator_control::_OpControlManes() {

 #if USING_15_INCH_ROBOT
  pros::lcd::print(1, "Angles rotated: %lf", encoder_back.get_position());
  double Forward = main_controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);    //Forward
  double Rotate = main_controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);  //Rotate

  // Forward = PreprocessJoysticks(Forward, 2, 0.0);

  // Rotate = PreprocessJoysticks(Rotate, 7, 0);

  // Forward or Backward (Left Joystick)
  left_side.moveVoltage(12000 * (Forward / 127));
  right_side.moveVoltage(12000 * (Forward / 127));
  
    // Turning (Right Joystick)
      // Turn right
  if (Rotate > 0){
    left_side.moveVoltage(12000 * (Rotate / 127));
    right_side.moveVoltage(12000 * (-Rotate / 127));
  }
      // Turn left
  else if (Rotate < 0){
    left_side.moveVoltage(12000 * (Rotate / 127));
    right_side.moveVoltage(12000 * (-Rotate / 127));
  }
  // Intake (X is positive, B is negative)
  int pos_intake = main_controller.get_digital(DIGITAL_A);
  int neg_intake = main_controller.get_digital(DIGITAL_B);

  if(pos_intake) intake.moveVoltage(12000 * pos_intake);
  else if(neg_intake) intake.moveVoltage(-12000 * neg_intake);
  else intake.moveVoltage(0);

  if (main_controller.get_digital(DIGITAL_X))
  {
    encoder_back.reset();
    encoder_back.reset_position();
  }
  

#else
  //pros::lcd::print(0, "X Position: %3lf", gps.get_status().x);
  //pros::lcd::print(0, "Y Position: %3lf", gps.get_status().y);

    //Tank
    // double Forward = main_controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)/127.0;    //Forward
    // double Rotate = main_controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)/127.0;    //Rotate
    // right_side.moveVoltage(12000 * Rotate);
    // left_side.moveVoltage(12000 * Rotate);

    //Normal
    double Forw = main_controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);   //Forward
    double Rotar = main_controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);  //Rotate

    Forw = PreprocessJoysticks(Forw, 2, 0.0);

    Rotar = PreprocessJoysticks(Rotar, 7, 0);

    left_side.moveVelocity(
      static_cast<int>(left_side.getGearing()) *
      std::clamp(Forw + Rotar, -1.0, 1.0));
    right_side.moveVelocity(
      static_cast<int>(right_side.getGearing()) *
      std::clamp(Forw - Rotar, -1.0, 1.0));

    if (main_controller.get_digital_new_press(DIGITAL_L1)) {
    if (!catapult_toggle) {
      catapult_toggle = true;
    } else if (proximity_sensor.get_value()) {
      catapult_toggle = false;
    }
    catapult_seen = false;
  }

  if (catapult_toggle && !proximity_sensor.get_value() && !catapult_seen) {
    catapult.moveVelocity(100);
  } else if (!catapult_toggle && proximity_sensor.get_value())
    catapult.moveVelocity(100);
  else
    catapult.moveVelocity(0);

  if (proximity_sensor.get_new_press()) catapult_seen = true;



   /* if(main_controller.get_digital(DIGITAL_Y))
    {
        intake.moveVoltage(12000);
    }*/

     if(main_controller.get_digital(DIGITAL_Y) && !main_controller.get_digital(DIGITAL_A))
  {
    expansion = !expansion;
    if(expansion)
    {
      valve_sole_eit.set_value(true);
    }
    else if(!expansion)
    {
      valve_sole_eit.set_value(false);
    }
    pros::delay(500);
  }

     if(main_controller.get_digital(DIGITAL_A) && !main_controller.get_digital(DIGITAL_Y))
  {
    expansion = !expansion;
    if(expansion)
    {
      solenoid_valve_eit.set_value(true);
    }
    else if(!expansion)
    {
      solenoid_valve_eit.set_value(false);
    }
    pros::delay(500);
  }


    /*if (main_controller.get_digital(DIGITAL_L1))
    {
        catapult.moveVoltage(8000);

        //Time to reset its prosition (varies with voltage)
        pros::delay(200);
    }
    else
    {
      catapult.moveVoltage(0);
    }*/

    // Intake
    if(main_controller.get_digital(DIGITAL_R1)) intake.moveVoltage(12000);
    else if(main_controller.get_digital(DIGITAL_R2)) intake.moveVoltage(-12000);
    else intake.moveVoltage(0);
    

#endif

}

inline void aon::operator_control::_OpControlDefault() { _OpControlManes(); }

// ============================================================================
//    ___          _
//   | _ ) ___  __| |_  _
//   | _ \/ _ \/ _` | || |
//   |___/\___/\__,_|\_, |
//                   |__/
// ============================================================================

inline void aon::operator_control::Run(const Drivers driver) {
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

#endif  // AON_COMPETITION_OPERATOR_CONTROL_HPP_