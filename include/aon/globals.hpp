#ifndef AON_GLOBALS_HPP_
#define AON_GLOBALS_HPP_

#include "../api.h"
#include "../okapi/api.hpp"
#include "./constants.hpp"
#include "controls/pid/pid.hpp"


#if USING_15_INCH_ROBOT
// Motor groups for drivetrain
okapi::MotorGroup driveLeft = okapi::MotorGroup({-1});
okapi::MotorGroup driveRight = okapi::MotorGroup({16});
okapi::MotorGroup driveFull = okapi::MotorGroup({16, -1});
// okapi::MotorGroup driveLeft = okapi::MotorGroup({18});
// okapi::MotorGroup driveRight = okapi::MotorGroup({-16});
// okapi::MotorGroup driveFull = okapi::MotorGroup({-18, 16});



okapi::MotorGroup intake = okapi::MotorGroup({-13, -14});
okapi::MotorGroup rail = okapi::MotorGroup({-13});
okapi::Motor gate = okapi::Motor(-14);

okapi::Motor arm = okapi::Motor(10);

okapi::Motor indexer = okapi::Motor(7);

//odometry
pros::Rotation encoderMid(12, true);
pros::Rotation encoderBack(11, false);

// Turret
okapi::Motor turret = okapi::Motor({20});
pros::Rotation turretEncoder(13, false);
pros::Vision vision_sensor(5); //14 in turret bot
pros::vision_signature_s_t RED_SIG = pros::Vision::signature_from_utility(1, 8973, 11143, 10058, -2119, -1053, -1586, 5.4, 0);
pros::vision_signature_s_t BLUE_SIG = pros::Vision::signature_from_utility(2, -3050, -2000, -2500, 8000, 11000, 9500, 5.4, 0);

pros::Gps gps(6, GPS_INITIAL_X, GPS_INITIAL_Y, GPS_INITIAL_HEADING, GPS_X_OFFSET, GPS_Y_OFFSET);
// pros::Gps gps(6, GPS_INITIAL_X, GPS_INITIAL_Y, GPS_INITIAL_HEADING);
// pros::Gps gps(6);
// Center of the field is (0,0), uses 4 quadrant cartesian system for coordinates
// 7.5 in = 0.1905 m
// approx 110 deg
// pros::Gps gps(6, 1.5, .3, 110, 0.1905, 0.1905); // measure the values to be certain

aon::PID drivePID = aon::PID(0.1, 0, 0);
aon::PID turnPID = aon::PID(0.01, 0, 0);
aon::PID fastPID = aon::PID(1, 0, 0);

pros::ADIDigitalIn limit_switch ('C');
pros::ADIDigitalIn dist_sensor ('B');
pros::Distance distanceSensor(2);
bool rail_on = false;

pros::ADIDigitalOut piston ('H');
bool piston_on = false;
bool indexerOut = false;

bool conveyor_auto = true;
int state = 0; // for railing

#if GYRO_ENABLED
pros::Imu gyroscope(15);
#endif

#else
// Set up motors and sensors for 18 inch robot
okapi::MotorGroup driveLeft = okapi::MotorGroup({12, -6, 19});
okapi::MotorGroup driveRight = okapi::MotorGroup({-15, 16, -17});
okapi::MotorGroup driveFull = okapi::MotorGroup({12, -15, 16, -17, -6, 19});

//SIGNATURE COLOR
pros::Vision vision_sensor(7);
pros::vision_signature_s_t RED_SIG = pros::Vision::signature_from_utility(1, 8973, 11143, 10058, -2119, -1053, -1586, 5.4, 0);
pros::vision_signature_s_t BLUE_SIG = pros::Vision::signature_from_utility(2, -3050, -2000, -2500, 8000, 11000, 9500, 5.4, 0);


pros::Gps gps(9, GPS_INITIAL_X, GPS_INITIAL_Y, GPS_INITIAL_HEADING, GPS_X_OFFSET, GPS_Y_OFFSET);

aon::PID drivePID = aon::PID(0.1, 0, 0);
aon::PID turnPID = aon::PID(0.01, 0, 0);
aon::PID fastPID = aon::PID(1, 0, 0);

pros::ADIDigitalIn limit_switch ('C');
pros::ADIDigitalIn dist_sensor ('B');
pros::Distance distanceSensor(10);
bool rail_on = false;

pros::ADIDigitalOut piston ('H');
bool piston_on = false;
bool indexerOut = false;
 
bool conveyor_auto = true;
int state = 0; // for railing

#if GYRO_ENABLED
pros::Imu gyroscope(5);
#endif

pros::Rotation encoderLeft(20, true); 
pros::Rotation encoderRight(11, true);

okapi::MotorGroup intake = okapi::MotorGroup({13, -14});
okapi::MotorGroup rail = okapi::MotorGroup({13});
okapi::Motor gate = okapi::Motor(-14);

okapi::Motor arm = okapi::Motor(10);

okapi::Motor indexer = okapi::Motor(8);

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

  driveFull.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  driveFull.setGearing(okapi::AbstractMotor::gearset::green);
  driveFull.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  driveFull.tarePosition();

  intake.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  intake.setGearing(okapi::AbstractMotor::gearset::green);
  intake.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  intake.tarePosition();

  arm.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
  arm.setGearing(okapi::AbstractMotor::gearset::red);
  arm.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  arm.tarePosition();

  indexer.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
  indexer.setGearing(okapi::AbstractMotor::gearset::green);
  indexer.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  indexer.tarePosition();

  turret.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
  turret.setGearing(okapi::AbstractMotor::gearset::green);
  turret.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  turret.tarePosition();

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

  driveFull.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  driveFull.setGearing(okapi::AbstractMotor::gearset::green);
  driveFull.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  driveFull.tarePosition();

  intake.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  intake.setGearing(okapi::AbstractMotor::gearset::green);
  intake.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  intake.tarePosition();

#endif
}

/**
 * \brief Toggles the value of a bool
 * 
 * \param boolean The variable to be toggled
 * 
 * \returns The updated boolean
 */
inline bool toggle(bool &boolean) {
  boolean = !boolean;
  return boolean;
}

/**
 * \brief Adds the colors to the vision sensor
*/
inline void ConfigureColors(){
  vision_sensor.set_signature(1, &RED_SIG);
  vision_sensor.set_signature(2, &BLUE_SIG);
}

/**
 * \brief Stops movement from robot
 */
void STOP(){
  driveFull.moveVelocity(0);
  intake.moveVelocity(0);
  arm.moveVelocity(0);
  indexer.moveVelocity(0);
  turret.moveVelocity(0);
}

/**
 * \brief Used to make sure a condition is being met or a line of code is being run
*/
void testEndpoint(int speed = 100){
  STOP();
  intake.moveVelocity(speed);
  pros::delay(1000);
  intake.moveVelocity(0);
}

}  // namespace aon

#endif  // AON_GLOBALS_HPP_
