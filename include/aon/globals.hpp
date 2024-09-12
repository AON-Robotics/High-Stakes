#ifndef AON_GLOBALS_HPP_
#define AON_GLOBALS_HPP_

#include "../api.h"
#include "../okapi/api.hpp"
#include "./constants.hpp"


// ============================================================================
//    ___         _       _
//   | _ \_ _ ___| |_ ___| |_ _  _ _ __  ___ ___
//   |  _/ '_/ _ \  _/ _ \  _| || | '_ \/ -_|_-<
//   |_| |_| \___/\__\___/\__|\_, | .__/\___/__/
//                            |__/|_|
// ============================================================================
namespace aon {

/**
 * \brief Helper function that configures motor gearings, units, etc.
 *
 * \details Helper function that configures every motor's brake mode, gearing,
 * and encoder units and finally resets the integrated encoders
 *
 */
inline void ConfigureMotors();

};  // namespace aon

// ============================================================================
//     ___           __ _                    _   _
//    / __|___ _ _  / _(_)__ _ _  _ _ _ __ _| |_(_)___ _ _
//   | (__/ _ \ ' \|  _| / _` | || | '_/ _` |  _| / _ \ ' \ 
//    \___\___/_||_|_| |_\__, |\_,_|_| \__,_|\__|_\___/_||_|
//                       |___/
// ============================================================================

pros::Controller main_controller (pros::E_CONTROLLER_MASTER);

//    _ ___
//   / | __|
//   | |__ \
//   |_|___/
//
#if USING_15_INCH_ROBOT

struct Coordinates{
double x;
double y;

Coordinates() {}
Coordinates(double a, double b) : x(a), y(b) {}
};

Coordinates rotated_pos;
int dampening_angle;
bool ending_rotation = false;

//pros::GPS gps = pros::GPS(15);

okapi::MotorGroup left_side = okapi::MotorGroup({6, 20});
okapi::MotorGroup right_side = okapi::MotorGroup ({-4, -11});
okapi::Motor intake = okapi::Motor(9);

pros::GPS gps = pros::GPS(13);

pros::ADIDigitalOut right_hand('A');
pros::ADIDigitalOut lift_valve('B');
pros::ADIDigitalOut left_hand('C');
bool extended_lift = false;
bool extended_left = false;
bool extended_right = false;

okapi::MotorGroup flywheel = okapi::MotorGroup({1});

// OLD
//okapi::Motor intake = okapi::Motor(-20);
//okapi::MotorGroup flywheel = okapi::MotorGroup({-2, 3});

okapi::Motor indexer = okapi::Motor(-8);

// pros::Vision vision_sensor(10);

pros::Rotation encoder_left(2, true);
pros::Rotation encoder_right(3, false);
pros::Rotation encoder_back(7, true);

pros::Imu gyro(8);

//pros::ADIDigitalOut expansion('B');

pros::ADIDigitalIn indexer_sensor('H');

#if GYRO_ENABLED
pros::Imu gyroscope(9);
#endif

//    _ ___
//   / ( _ )
//   | / _ \
//   |_\___/
//
#else

//15 for testing
pros::ADIDigitalIn proximity_sensor('A');
pros::ADIDigitalOut solenoid_valve_eit('B');
pros::ADIDigitalOut valve_sole_eit('C');
bool expansion = false;
okapi::MotorGroup left_side = okapi::MotorGroup({11, 12, 13});
okapi::MotorGroup right_side = okapi::MotorGroup({-20, -19, -18});
bool test_stop = false;
pros::GPS gps = pros::GPS(20);
inline bool catapult_toggle = false;
inline bool catapult_seen = false;

//18


struct Coordinates{
double x;
double y;

Coordinates() {}
Coordinates(double a, double b) : x(a), y(b) {}
};

Coordinates rotated_pos;
int dampening_angle;
bool ending_rotation = false;

// okapi::MotorGroup left_side = okapi::MotorGroup({11, 12, 13});
// okapi::MotorGroup right_side = okapi::MotorGroup({-18, -19, -20});
okapi::MotorGroup catapult = okapi::MotorGroup({14, -17});
okapi::Motor intake = okapi::Motor(10);

// pros::GPS gps = pros::GPS(1);

pros::Rotation encoder_left(2, false);
pros::Rotation encoder_right(8, true);
pros::Rotation encoder_back(16, true);

pros::ADIDigitalIn indexer_sensor('H');

#endif

#if GYRO_ENABLED
pros::Imu gyroscope(9);
#endif

namespace aon::operator_control{

//    _ ___
//   / | __|
//   | |__ \
//   |_|___/
//
#if USING_15_INCH_ROBOT

inline double flywheel_rpm = 350;
inline double flywheel_tbh = 4300;
const double flywheel_tbh_gain = 1.5;

//    _ ___
//   / ( _ )
//   | / _ \
//   |_\___/
//
#else

inline double flywheel_rpm = 380;
inline double flywheel_tbh = 4300;
const double flywheel_tbh_gain = 1.5;

#endif

/// Driver profiles for all robots
enum Drivers {
  kEnrique,
  kManes,
  kDiego,
  kDefault,
};
}
//namespace aon::operator_control

// ============================================================================
//    ___          _
//   | _ ) ___  __| |_  _
//   | _ \/ _ \/ _` | || |
//   |___/\___/\__,_|\_, |
//                   |__/
// ============================================================================

inline void aon::ConfigureMotors() {
//    _ ___
//   / | __|
//   | |__ \
//   |_|___/
//
#if USING_15_INCH_ROBOT

  left_side.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  left_side.setGearing(okapi::AbstractMotor::gearset::green);
  left_side.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  left_side.tarePosition();

  right_side.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  right_side.setGearing(okapi::AbstractMotor::gearset::green);
  right_side.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  right_side.tarePosition();

  flywheel.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  flywheel.setGearing(okapi::AbstractMotor::gearset::green);
  flywheel.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  flywheel.tarePosition();

  intake.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  intake.setGearing(okapi::AbstractMotor::gearset::green);
  intake.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  intake.tarePosition();

  indexer.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  indexer.setGearing(okapi::AbstractMotor::gearset::green);
  indexer.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  indexer.tarePosition();

//    _ ___
//   / ( _ )
//   | / _ \
//   |_\___/
//
#else

  left_side.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  left_side.setGearing(okapi::AbstractMotor::gearset::green);
  left_side.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  left_side.tarePosition();

  right_side.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  right_side.setGearing(okapi::AbstractMotor::gearset::green);
  right_side.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  right_side.tarePosition();

  // drive_left_up.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  // drive_left_up.setGearing(okapi::AbstractMotor::gearset::green);
  // drive_left_up.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  // drive_left_up.tarePosition();

  // drive_right_up.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  // drive_right_up.setGearing(okapi::AbstractMotor::gearset::green);
  // drive_right_up.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  // drive_right_up.tarePosition();

  // drive_left_down.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  // drive_left_down.setGearing(okapi::AbstractMotor::gearset::green);
  // drive_left_down.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  // drive_left_down.tarePosition();

  // drive_right_down.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  // drive_right_down.setGearing(okapi::AbstractMotor::gearset::green);
  // drive_right_down.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  // drive_right_down.tarePosition();

  // flywheel.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  // flywheel.setGearing(okapi::AbstractMotor::gearset::green);
  // flywheel.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  // flywheel.tarePosition();

  catapult.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  catapult.setGearing(okapi::AbstractMotor::gearset::green);
  catapult.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  catapult.tarePosition();

  // indexer.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  // indexer.setGearing(okapi::AbstractMotor::gearset::green);
  // indexer.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  // indexer.tarePosition();

#endif
}

#endif  // AON_GLOBALS_HPP_