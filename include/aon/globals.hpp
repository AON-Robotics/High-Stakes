#ifndef AON_GLOBALS_HPP_
#define AON_GLOBALS_HPP_

#include "../api.h"
#include "../okapi/api.hpp"
#include "./constants.hpp"
#include "controls/pid/pid.hpp"
#include "../pros/vision.hpp"

#if USING_15_INCH_ROBOT
// Motor groups for drivetrain
okapi::MotorGroup driveLeft = okapi::MotorGroup({10, -9, 8});
okapi::MotorGroup driveRight = okapi::MotorGroup({-20, 19, -18});

okapi::MotorGroup intake = okapi::MotorGroup({-17, -11, -2});
okapi::Motor gate = okapi::Motor(-2);
okapi::MotorGroup rail = okapi::MotorGroup({-17, -11});

// Rotation sensors for odometry
pros::Rotation encoderLeft(1, true);
pros::Rotation encoderRight(10, true);
pros::Rotation encoderBack(18, false);

//Distance Sensor
pros::Distance Distance_Sensor(13);

//Vision Sensor
pros::Vision Vision_Sensor(1);
// Red color variants
pros::vision_signature_s_t Light_Red_SIG = pros::vision_signature_s_t(pros::Vision::signature_from_utility(1, 3000, 6000, 4500, 5000, 7000, 6000, 3.000, 0));
pros::vision_signature_s_t Medium_Light_Red_SIG = pros::vision_signature_s_t(pros::Vision::signature_from_utility(2, 4000, 7000, 5500, 4000, 6000, 5000, 3.500, 0));
pros::vision_signature_s_t Medium_Red_SIG = pros::vision_signature_s_t(pros::Vision::signature_from_utility(3, 5000, 8000, 6500, 3500, 5500, 4500, 4.000, 0));
pros::vision_signature_s_t Medium_Dark_Red_SIG = pros::vision_signature_s_t(pros::Vision::signature_from_utility(4, 6000, 9000, 7500, 3000, 5000, 4000, 4.500, 0));
pros::vision_signature_s_t Dark_Red_SIG = pros::vision_signature_s_t(pros::Vision::signature_from_utility(5, 7000, 10000, 8500, 2000, 4000, 3000, 5.000, 0));
// Blue color variants
pros::vision_signature_s_t Light_Blue_SIG = pros::vision_signature_s_t(pros::Vision::signature_from_utility(6, -3000, -1500, -2250, 4000, 7000, 5500, 3.000, 0));
pros::vision_signature_s_t Medium_Light_Blue_SIG = pros::vision_signature_s_t(pros::Vision::signature_from_utility(7, -3500, -2000, -2750, 3500, 6500, 5000, 3.500, 0));
pros::vision_signature_s_t Medium_Blue_SIG = pros::vision_signature_s_t(pros::Vision::signature_from_utility(8, -4000, -2500, -3250, 3000, 6000, 4500, 4.000, 0));
pros::vision_signature_s_t Medium_Dark_Blue_SIG = pros::vision_signature_s_t(pros::Vision::signature_from_utility(9, -4500, -3000, -3750, 2500, 5500, 4000, 4.500, 0));
pros::vision_signature_s_t Dark_Blue_SIG = pros::vision_signature_s_t(pros::Vision::signature_from_utility(10, -5000, -3500, -4250, 2000, 5000, 3500, 5.000, 0));
// Color codes
pros::vision_color_code_t Red = Vision_Sensor.create_color_code(1, 2, 3, 4, 5);
pros::vision_color_code_t Blue = Vision_Sensor.create_color_code(6, 7, 8, 9, 10);


// TEMPORARY PORT THERE IS NO GPS INSTALLED YET
pros::Gps gps(3, -0.127, -0.1397);

aon::PID drivePID = aon::PID(0.1, 0, 0);
aon::PID turnPID = aon::PID(0.01, 0, 0);

pros::ADIDigitalIn limit_switch ('C');
bool rail_on = false;

pros::ADIDigitalOut piston ('A');
bool piston_on = false;
 
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
