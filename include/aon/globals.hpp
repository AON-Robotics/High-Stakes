#ifndef AON_GLOBALS_HPP_
#define AON_GLOBALS_HPP_

#include "../api.h"
#include "../okapi/api.hpp"
#include "./constants.hpp"

#if USING_15_INCH_ROBOT
// Motor groups for drivetrain
okapi::MotorGroup driveLeft = okapi::MotorGroup({11, 12});
okapi::MotorGroup driveRight = okapi::MotorGroup({-19, -20});

// Rotation sensors for odometry
pros::Rotation encoderLeft(1, true);
pros::Rotation encoderRight(10, true);
pros::Rotation encoderBack(18, false);

// TEMPORARY PORT THERE IS NO INTAKE INSTALLED YET
okapi::Motor intake = okapi::Motor(2);

// TEMPORARY PORT THERE IS NO GPS INSTALLED YET
pros::Gps gps(3);

#else
// Set up motors and sensors for 18 inch robot3)
okapi::MotorGroup driveLeft = okapi::MotorGroup({-12, 11});
okapi::MotorGroup drive_back_left = okapi::MotorGroup({-13, 14});
okapi::MotorGroup driveRight = okapi::MotorGroup({19, -20});
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
  driveLeft.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  driveLeft.setGearing(okapi::AbstractMotor::gearset::blue);
  driveLeft.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  driveLeft.tarePosition();

  driveRight.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  driveRight.setGearing(okapi::AbstractMotor::gearset::blue);
  driveRight.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  driveRight.tarePosition();

  intake.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  intake.setGearing(okapi::AbstractMotor::gearset::blue);
  intake.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  intake.tarePosition();

#else
  // Configure motors for 18 inch robot
  driveLeft.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  driveLeft.setGearing(okapi::AbstractMotor::gearset::green);
  driveLeft.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  driveLeft.tarePosition();

  driveRight.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  driveRight.setGearing(okapi::AbstractMotor::gearset::green);
  driveRight.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  driveRight.tarePosition();

  intake.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  intake.setGearing(okapi::AbstractMotor::gearset::blue);
  intake.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  intake.tarePosition();

#endif
}

}  // namespace aon

#endif  // AON_GLOBALS_HPP_
