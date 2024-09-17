#ifndef AON_GLOBALS_HPP_
#define AON_GLOBALS_HPP_

#include "../api.h"
#include "../okapi/api.hpp"
#include "./constants.hpp"

#if USING_15_INCH_ROBOT
// Set up motors and sensors for 15 inch robot
okapi::MotorGroup drive_front_left = okapi::MotorGroup({-18, 19});
okapi::MotorGroup drive_back_left = okapi::MotorGroup({16, -17});
okapi::MotorGroup drive_front_right = okapi::MotorGroup({-11, 12});
okapi::MotorGroup drive_back_right = okapi::MotorGroup({-13, 14});

okapi::Motor intake = okapi::Motor(-20);
okapi::MotorGroup flywheel = okapi::MotorGroup({-2, 3});
okapi::Motor indexer = okapi::Motor(-8);

pros::Rotation encoder_left(9, true);
pros::Rotation encoder_right(1, false);
pros::Rotation encoder_back(10, true);

pros::Gps gps(20);

pros::ADIDigitalIn proximity_sensor('A');
pros::ADIDigitalOut expansion('B');

#else
// Set up motors and sensors for 18 inch robot3)
okapi::MotorGroup drive_front_left = okapi::MotorGroup({-12, 11});
okapi::MotorGroup drive_back_left = okapi::MotorGroup({-13, 14});
okapi::MotorGroup drive_front_right = okapi::MotorGroup({19, -20});
okapi::MotorGroup drive_back_right = okapi::MotorGroup({18, -17});

okapi::MotorGroup intake = okapi::MotorGroup({5, 3});  // needs updates
okapi::MotorGroup flywheel = okapi::MotorGroup({-9, 10});
okapi::Motor puncher = okapi::Motor(1);  // need updates

pros::Rotation encoder_left(3, false);
pros::Rotation encoder_right(8, true);
pros::Rotation encoder_back(16, false);

pros::ADIDigitalOut expansion('B');

#if GYRO_ENABLED
pros::Imu gyroscope(9);
#endif

#endif

namespace aon::operator_control {
inline double flywheel_on = false;
const double flywheel_rpm_increment = 10;
inline double flywheel_tbh_last_error = 0;
inline double flywheel_tbh_error = 0;
inline double flywheel_tbh_output = 0;

#if USING_15_INCH_ROBOT

inline double flywheel_rpm = 480;
inline double flywheel_tbh = 4300;
const double flywheel_tbh_gain = 1.5;

#else

inline double flywheel_rpm = 480;
inline double flywheel_tbh = 4300;
const double flywheel_tbh_gain = 1.5;

#endif

/// Driver profiles for all robots
enum Drivers {
  kEnrique,
  kManes,
  kDefault,
};
}  // namespace aon::operator_control

pros::Controller main_controller = pros::Controller(pros::E_CONTROLLER_MASTER);

namespace aon {

inline void ConfigureMotors() {
#if USING_15_INCH_ROBOT
  // Configure motors for 15 inch robot
  drive_front_left.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  drive_front_left.setGearing(okapi::AbstractMotor::gearset::blue);
  drive_front_left.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  drive_front_left.tarePosition();

  drive_front_right.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  drive_front_right.setGearing(okapi::AbstractMotor::gearset::blue);
  drive_front_right.setEncoderUnits(
      okapi::AbstractMotor::encoderUnits::degrees);
  drive_front_right.tarePosition();

  drive_back_left.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  drive_back_left.setGearing(okapi::AbstractMotor::gearset::blue);
  drive_back_left.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  drive_back_left.tarePosition();

  drive_back_right.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  drive_back_right.setGearing(okapi::AbstractMotor::gearset::blue);
  drive_back_right.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  drive_back_right.tarePosition();

  flywheel.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  flywheel.setGearing(okapi::AbstractMotor::gearset::blue);
  flywheel.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  flywheel.tarePosition();

  intake.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  intake.setGearing(okapi::AbstractMotor::gearset::blue);
  intake.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  intake.tarePosition();

#else
  // Configure motors for 18 inch robot
  drive_front_left.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  drive_front_left.setGearing(okapi::AbstractMotor::gearset::green);
  drive_front_left.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  drive_front_left.tarePosition();

  drive_front_right.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  drive_front_right.setGearing(okapi::AbstractMotor::gearset::green);
  drive_front_right.setEncoderUnits(
      okapi::AbstractMotor::encoderUnits::degrees);
  drive_front_right.tarePosition();

  drive_back_left.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  drive_back_left.setGearing(okapi::AbstractMotor::gearset::green);
  drive_back_left.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  drive_back_left.tarePosition();

  drive_back_right.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  drive_back_right.setGearing(okapi::AbstractMotor::gearset::green);
  drive_back_right.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  drive_back_right.tarePosition();

  flywheel.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  flywheel.setGearing(okapi::AbstractMotor::gearset::blue);
  flywheel.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  flywheel.tarePosition();

  puncher.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
  puncher.setGearing(okapi::AbstractMotor::gearset::red);
  puncher.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  puncher.tarePosition();

  intake.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  intake.setGearing(okapi::AbstractMotor::gearset::blue);
  intake.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  intake.tarePosition();

#endif
}

}  // namespace aon

#endif  // AON_GLOBALS_HPP_
