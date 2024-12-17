#ifndef AON_GLOBALS_HPP_
#define AON_GLOBALS_HPP_

#include "../api.h"
#include "../okapi/api.hpp"
#include "./constants.hpp"
#include "controls/pid/pid.hpp"

#if USING_15_INCH_ROBOT
// Motor groups for drivetrain
okapi::MotorGroup driveLeft = okapi::MotorGroup({10, -9, 8});
okapi::MotorGroup driveRight = okapi::MotorGroup({-20, 19, -18});

okapi::MotorGroup intake = okapi::MotorGroup({-17, -11, -2});
okapi::Motor gate = okapi::Motor(-2);
okapi::MotorGroup rail = okapi::MotorGroup({-17, -11});
//Vision sensor port 
pros::Vision visual_sensor(14);
pros::vision_signature_s_t Green_SIG= pros::vision_signature_s_t(pros::Vision::signature_from_utility(1, -4729, -3713, -4221, -3483, -2603, -3043, 4.600, 0));
//vision::signature GR (1, -4729, -3713, -4221, -3483, -2603, -3043, 4.600, 0);

// Rotation sensors for odometry
pros::Rotation encoderLeft(1, true);
pros::Rotation encoderRight(10, true);
pros::Rotation encoderBack(18, false);

// TEMPORARY PORT THERE IS NO GPS INSTALLED YET
pros::Gps gps(3, -0.127, -0.1397);

aon::PID drivePID = aon::PID(0.1, 0, 0);
aon::PID turnPID = aon::PID(0.01, 0, 0);

pros::ADIDigitalIn limit_switch ('C');
pros::ADIDigitalIn dist_sensor ('B');
bool rail_on = false;

pros::ADIDigitalOut piston ('A');
bool piston_on = false;
 
bool conveyor_auto = true;
int state = 0;

#if GYRO_ENABLED
pros::Imu gyroscope(2);
#endif

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
  driveLeft.setGearing(okapi::AbstractMotor::gearset::green);
  driveLeft.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  driveLeft.tarePosition();

  driveRight.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  driveRight.setGearing(okapi::AbstractMotor::gearset::green);
  driveRight.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  driveRight.tarePosition();

  intake.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  intake.setGearing(okapi::AbstractMotor::gearset::green);
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
