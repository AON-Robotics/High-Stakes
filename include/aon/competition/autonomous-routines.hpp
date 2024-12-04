#pragma once

#include <cmath>
#include <algorithm>
#include "../constants.hpp"
#include "../globals.hpp"
#include "../sensing/odometry.hpp"
#include "../controls/pid/pid.hpp"
#include "../controls/holonomic-motion.hpp"

namespace aon {

#define SAMPLE_SIZE 50

double initial_pos_x;
double initial_pos_y;
double initial_heading;

#if USING_15_INCH_ROBOT

/**
 * \brief Calculates time for the robot to reach a given distance
 * 
 * \param distance Distance from the robot to the target (remains constant) in \b inches
 * \returns The approximate time necessary to reach the target (overestimation) in \b seconds
 */
inline double getTimetoTarget(const double &distance){
  double circumference = DRIVE_WHEEL_DIAMETER * M_PI; // of the drive wheel (inches)
  double RPS = (int)driveRight.getGearing() / 60; // (revolutions per seconds)
  double velocity = RPS * circumference; // Calculated speed (in / s)
  double time = 2 * distance / velocity; // Calculated time (seconds)
  return time;
}

/**
 * \brief Calculates time for the robot to turn an angle
 * 
 * \param radians Angle remaining from the robot's current angle to the target (remains constant) in \b radians
 * 
 * \details The arc length formula is used as s = theta * radius (theta in radians)
 * 
 * \returns The approximate time necessary to reach the target (overestimation) in \b seconds
 * 
 */
inline double getTimetoTurnRad(const double &radians){
  double arcLength = radians * AVG_DRIVETRAIN_RADIUS; // Of the turn (inches)
  double circumference = DRIVE_WHEEL_DIAMETER * M_PI; // of the drive wheel (inches)
  double RPS = (int)driveRight.getGearing() / 60; // (revolutions per seconds)
  RPS /= 2; // We are using half power to turn
  double velocity = RPS * circumference; // Calculated speed (in / s)
  double time = 2 * arcLength / velocity; // Calculated time (seconds)
  return time;
}

/**
 * \brief Calculates time for the robot to turn an angle
 * 
 * \param degrees Angle remaining from the robot's current angle to the target (remains constant) in \b degrees
 * 
 * \returns The approximate time necessary to reach the target (overestimation) in \b seconds
 * 
 */
inline double getTimetoTurnDeg(const double &degrees) { return getTimetoTurnRad(degrees * M_PI / 180); }

/**
 * \brief Moves the robot toward a given position (default forward)
 * 
 * \param pid The PID used for the driving
 * \param targetPos A vector representing the position towards which we want to go
 * \param sign Determines the direction of the movement, 1 is front and -1 is backwards
 * 
 */

void MoveDrivePID(aon::PID pid, aon::Vector targetPos, double sign = 1) {

  // double avg_x = 0;
  // double avg_y = 0;
  
  // Get the average of the readings from the GPS
  // for (int i = 0; i < SAMPLE_SIZE; i++)
  // {
  //   avg_x += gps.get_status().x;
  //   avg_y += gps.get_status().y;
  // }

  // avg_x /= SAMPLE_SIZE; avg_y /= SAMPLE_SIZE;
  // End average


  // Define the initialPos using the GPS instead of odometry (later should be both)
  // aon::Vector initialPos = Vector().SetPosition(avg_x, avg_y);
  pid.Reset();
  aon::Vector initialPos = aon::odometry::GetPosition();

  const double timeLimit = getTimetoTarget(std::abs((targetPos - initialPos).GetMagnitude()));
  const double start_time = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - start_time //every time the variable is called it is recalculated automatically

  //Target position minus initial position
  const double targetDiplacement = (targetPos - initialPos).GetMagnitude();
  while (time < timeLimit) {
    aon::odometry::Update();

    double currentDisplacement = (aon::odometry::GetPosition() - initialPos).GetMagnitude();

    double output = pid.Output(targetDiplacement, currentDisplacement);

    pros::lcd::print(0, "%f", currentDisplacement);

    driveLeft.moveVelocity(sign * std::clamp(output * (int)driveLeft.getGearing(), -100.0, 100.0));
    driveRight.moveVelocity(sign * std::clamp(output * (int)driveLeft.getGearing(), -100.0, 100.0));

    pros::delay(10);
  }

  // Stop the movement
  driveLeft.moveVelocity(0);
  driveRight.moveVelocity(0);
#undef time
}

/**
 * \brief Moves the robot a given distance (default forward)
 * 
 * \param pid The PID used for the driving
 * \param dist The distance to be moved in \b inches
 */

void MoveDrivePID(aon::PID pid, double dist) {
  const int sign = dist / abs(dist); // Getting the direction of the movement
  dist = abs(dist); // Setting the magnitude to positive

  // double avg_x = 0;
  // double avg_y = 0;
  
  // Get the average of the readings from the GPS
  // for (int i = 0; i < SAMPLE_SIZE; i++)
  // {
  //   avg_x += gps.get_status().x;
  //   avg_y += gps.get_status().y;
  // }

  // avg_x /= SAMPLE_SIZE; avg_y /= SAMPLE_SIZE;
  // End average


  // Define the initialPos using the GPS instead of odometry (later should be both)
  // aon::Vector initialPos = Vector().SetPosition(avg_x, avg_y);
  pid.Reset();
  aon::Vector initialPos = aon::odometry::GetPosition();

  const double timeLimit = getTimetoTarget(dist);
  const double start_time = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - start_time //every time the variable is called it is recalculated automatically

  while (time < timeLimit) {
    aon::odometry::Update();

    double currentDisplacement = (aon::odometry::GetPosition() - initialPos).GetMagnitude();

    double output = pid.Output(dist, currentDisplacement);

    pros::lcd::print(0, "%f", currentDisplacement);

    driveLeft.moveVelocity(sign * std::clamp(output * (int)driveLeft.getGearing(), -100.0, 100.0));
    driveRight.moveVelocity(sign * std::clamp(output * (int)driveLeft.getGearing(), -100.0, 100.0));

    pros::delay(10);
  }

  // Stop the motors
  driveLeft.moveVelocity(0);
  driveRight.moveVelocity(0);
#undef time
}

/**
 * \brief Turns the robot by a given angle (default clockwise)
 * 
 * \param pid The PID to be used for the turn
 * \param angle The angle to make the robot turn in \b degrees
 */
void MoveTurnPID(PID pid, double angle){
  const int sign = angle/abs(angle); // Getting the direction of the movement
  angle = abs(angle); // Setting the magnitude to positive
  pid.Reset();
  gyroscope.tare(); // .tare() or .reset(true) depending on the time issue
  const double startAngle = gyroscope.get_heading(); // Angle relative to the start

  if(sign == -1) { angle = 360.0 - angle + CLOCKWISE_ROTATION_DEGREES_OFFSET; }
  if(sign == 1) { angle -= CLOCKWISE_ROTATION_DEGREES_OFFSET; }

  const double targetAngle = angle;

  double timeLimit = getTimetoTurnDeg(targetAngle);

  const double startTime = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - startTime

  while(time < timeLimit){
    aon::odometry::Update();

    double traveledAngle = gyroscope.get_heading() - startAngle;

    double output = std::abs(pid.Output(targetAngle, traveledAngle)); //Use the absolute value of the output because if not, counter-clockwise turning is weird (error)

    pros::lcd::print(0, "%f", traveledAngle);

    // Taking clockwise rotation as positive (to change this just flip the negative on the sign below)
    driveLeft.moveVelocity(sign * std::clamp(output * (int)driveLeft.getGearing(), -50.0, 50.0));
    driveRight.moveVelocity(-sign * std::clamp(output * (int)driveRight.getGearing(), -50.0, 50.0));

    pros::delay(10);
  }

  driveLeft.moveVelocity(0);
  driveRight.moveVelocity(0);

  #undef time
}

/**
 * \brief Turns the robot clockwise by 90
 * 
 * \param pid The PID to be used for the turn
 * \param amt The angle to make the robot turn in \b degrees
 * \param sign 1 when turning clockwise and -1 when turning counter-clockwise
 */
void turn90(PID pid, int amt = 1, double sign = 1){
  const double startAngle = gyroscope.get_heading(); // Angle relative to the start

  double targetAngle = 90 * amt;
  
  if(sign == -1) { targetAngle = 360 - targetAngle; }

  double timeLimit = getTimetoTurnRad(amt * M_PI / 2); 
  const double startTime = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - startTime

  while(time < timeLimit){
    // aon::odometry::Update();

    double traveledAngle = gyroscope.get_heading() - startAngle;

    double output = std::abs(pid.Output(targetAngle, traveledAngle));

    pros::lcd::print(0, "%f", traveledAngle);

    // Taking clockwise rotation as positive (to change this just flip the negative on the sign below)
    driveLeft.moveVelocity(sign * std::clamp(output * (int)driveLeft.getGearing(), -50.0, 50.0));
    driveRight.moveVelocity(-sign * std::clamp(output * (int)driveRight.getGearing(), -50.0, 50.0));

    pros::delay(10);
  }

  driveLeft.moveVelocity(0);
  driveRight.moveVelocity(0);

  #undef time
}



inline void first_routine(double kP, double kI, double kD) {
  aon::PID pid = aon::PID(kP, kI, kD);
  int dist = 36; // Inches
  aon::Vector target = aon::Vector().SetPosition(dist, 0);
  
  aon::MoveDrivePID(pid, target);
  pid.Reset();

  intake.moveVelocity(90);
  pros::delay(500);
  intake.moveVelocity(0);

  target = aon::Vector().SetPosition(0, 0);
  aon::MoveDrivePID(pid, target, -1);

  pros::delay(500);
  // aon::turn90(pid);


  pid.Reset();
}

inline void turnTest(double kP, double kI, double kD) {
  PID pid = PID(kP, kI, kD);
  for(int i = 0; i < 4; i++){
    aon::MoveTurnPID(pid, 90);
    pros::delay(500);
  }
  // aon::MoveTurnPID(pid, 90, -1);
  // pros::delay(500);
  // aon::MoveTurnPID(pid, 45, -1);
  // pros::delay(500);
  pid.Reset();
}

inline void programming_skills() {
  first_routine(10, 0, 0);
  // drive_left.moveVelocity(-100);
  // drive_right.moveVelocity(-100);
  pros::delay(1500);
  // drive_right.moveVelocity(0);
  // aon::PID pid = aon::PID(10, 0, 0);
  // aon::Vector target = aon::Vector().SetPosition(-2.0, 0);
  // aon::MoveDrivePID(pid, target, 2);
  // pid.Reset();

  /*target = aon::Vector().SetPosition(0, 0);
  aon::MoveDrivePID(f_pid, target, 3, -1);
  f_id.Reset();*/

  // expansion.set_value(1);

}
  int first_routine_wrapper() {  // fixing gui return type Temp
  // aon::odometry::Debug();
  first_routine(10, 0, 0);
  return 0;
  
}

void tempRoutine() {  // temporary routne to test GUI

  // aon::PID x_pid = aon::PID(10, 0, 0.75);
  // aon::PID y_pid = aon::PID(10, 0, 0.75);
  // aon::PID heading_pid = aon::PID(0.15, 0, 0.0099);

  // MoveDrivePID(x_pid, y_pid, heading_pid, 10, 0, 0, 3);
  // x_pid.Reset();
  // y_pid.Reset();
  // heading_pid.Reset();
}

int tempRoutine_wrapper() {  // fixing gui return type Temp
  // aon::odometry::Debug();
  // tempRoutine();
  aon::programming_skills();
  // driveLeft.moveVoltage(12000);
  // driveRight.moveVoltage(12000);
  return 0;
}

// TESTING PHASE FOR PID

int proportionalFavoredRoutine(){
  first_routine(10, 0, 0);
  return 0;
}

int integralFavoredRoutine(){
  first_routine(0, 10, 0);
  return 0;
}

// This one causes no movement because it depends on change of error (there is none)
int derivativeFavoredRoutine(){
  first_routine(0, 0, 10);
  return 0;
}

int initialReset()
{
  //3 seconds
  odometry::ResetInitial();
  gyroscope.reset(true);
  return 1;
}

int move(double dist)
{
  //Const time
  MoveDrivePID(drivePID, dist);
  drivePID.Reset();
  turnPID.Reset();
  // pros::delay(500);
  return 1;
}

int turn(double rot)
{
  // gyroscope.reset(true);
  // pros::delay(3000);
  MoveTurnPID(turnPID, rot);
  drivePID.Reset();
  turnPID.Reset();
  // pros::delay(500);
  return 1;
}

void squareRoutine(){
  PID drivePID = PID(0.1, 0, 0);
  PID turnPID = PID(0.01, 0, 0);
  const int dist = 12;
  //Draws a square with the robot
  for(int i = 0; i < 4; i++){
    odometry::ResetInitial();
    MoveDrivePID(drivePID, Vector().SetPosition(dist, 0));
    // drivePID.Reset();
    // turnPID.Reset();

    // gyroscope.reset(true);
    MoveTurnPID(turnPID, 90);
    // drivePID.Reset();
    // turnPID.Reset();

    pros::delay(500);
  }
}

int combinationPIDRoutine(){
  // squareRoutine();
  // first_routine(0.1, 0, 0);
  turnTest(0.01, 0, 0);
  return 0;
}

// END TEST FUNCTIONS


#else
void MoveDrivePID(aon::PID x_pid, aon::PID y_pid, aon::PID heading_pid,
                  double set_x, double set_y, double set_heading,
                  double timeout, double position_tolerance = -1,
                  double max_x_speed = INFINITY, double max_y_speed = INFINITY,
                  double max_heading_speed = INFINITY) {
  const double start_time = pros::micros() / 1E6;
#define t (pros::micros() / 1E6 - start_time)
#define DELTA_POS std::hypot(x_pid.GetError(), y_pid.GetError())

  while (t < timeout) {
    pros::lcd::print(3, "t= %F", t);
    aon::odometry::Update();

    x_pid.Output(set_x, aon::odometry::GetX());
    y_pid.Output(set_y, aon::odometry::GetY());
    heading_pid.Output(set_heading, -aon::odometry::GetDegrees());

    aon::holonomic_motion::MoveHolonomicMotion(
        std::clamp(x_pid.GetResult(), -max_x_speed, max_x_speed),
        std::clamp(y_pid.GetResult(), -max_y_speed, max_y_speed),
        std::clamp(heading_pid.GetResult(), -max_heading_speed,
                   max_heading_speed));
    pros::lcd::print(0, "x_error %f", x_pid.GetError());
    pros::lcd::print(1, "y_error %f", y_pid.GetError());
    pros::lcd::print(2, "heading_error %f", heading_pid.GetError());

    pros::delay(10);
  }
  pros::lcd::print(4, "t= %F", t);
  aon::holonomic_motion::MoveHolonomicMotion(0, 0, 0);
#undef t
#undef DELTA_POS
}

void tbh() {
  double last_error = 0, error = 0, output = 0, tbh = 4300;
  double set_point = 200;
  const double gain = 1.5;

  // const double gain = 15;
  // Gradually accelerate until the tbh voltage
  // for (int i = 0; i < tbh; i += tbh / 200.0) {
  //   flywheel.moveVoltage(i);
  //   pros::delay(10);
  // }

  // Begin tbh controller
  while (1) {
    error = set_point - flywheel.getActualVelocity();
    output += gain * error;

    if (std::signbit(error) != std::signbit(last_error)) {
      output = 0.5 * (output + tbh);
      tbh = output;
      last_error = error;
    }

    flywheel.moveVoltage(output);
    pros::lcd::print(0, "RPM = %0.2f rpm", flywheel.getActualVelocity());
    pros::lcd::print(1, "Output = %0.2f mV", output);
    pros::lcd::print(2, "tbh = %0.2f mV", tbh);
    pros::lcd::print(3, "error = %0.2f rpm", error);

    pros::delay(10);
  }
}

void first_routine() {
  // aon::odometry::Debug();
  //   aon::PID x_pid = aon::PID(10, 0, 0.75, 0.010, 2, 20);
  //   aon::PID y_pid = aon::PID(10, 0, 0.75, 0.010, 2, 34);
  //   aon::PID heading_pid = aon::PID(0.075, 0.15, 0, 0.010, 20, 5);
  //  aon::PID x_pid = aon::PID(6, 7, 0.5, 0.010, 3, 10);
  //  aon::PID y_pid = aon::PID(6, 7, 0.5, 0.010, 3, 10);
  //  aon::PID heading_pid = aon::PID(0.15, 0.267, 0.009, 0.010, 20, 5);

  aon::PID x_pid = aon::PID(10, 0, 0.75);
  aon::PID y_pid = aon::PID(10, 0, 0.75);
  aon::PID heading_pid = aon::PID(0.15, 0, 0.0099);
  // rolo x-18, y3.5, 0

  // MoveDrivePID(x_pid, y_pid, heading_pid, -23, 0, 0, 3);
  MoveDrivePID(x_pid, y_pid, heading_pid, -10, 0, 0, 3);
  x_pid.Reset();
  y_pid.Reset();
  heading_pid.Reset();
  // aon::odometry::Debug();
  //  pros::lcd::print(0,"errorX %f", x_pid.GetError());

  // intake.moveVoltage(12000);
  // //flywheel.moveVoltage(9500);
  // pros::delay(1000);

  // MoveDrivePID(x_pid, y_pid, heading_pid, -23, 3.6, 0, 2.75);
  // x_pid.Reset();
  // y_pid.Reset();
  // heading_pid.Reset();

  // intake.moveVelocity(-75);
  // // flywheel.moveVoltage(10000);
  // pros::delay(500);
  // intake.moveVelocity(0);

  // MoveDrivePID(x_pid, y_pid, heading_pid, -23, 2, 0, 2.75);
  // x_pid.Reset();
  // y_pid.Reset();
  // heading_pid.Reset();

  flywheel.moveVoltage(8000);

  MoveDrivePID(x_pid, y_pid, heading_pid, -23, 2, -90, 2.75);
  x_pid.Reset();
  y_pid.Reset();
  heading_pid.Reset();

  puncher.moveRelative(720, 75);
  // pros::delay(2000);
  // aon::odometry::Debug();

  // MoveDrivePID(x_pid, y_pid, heading_pid, -26, -23, 170, 3);
  // x_pid.Reset();
  // y_pid.Reset();
  // heading_pid.Reset();

  // MoveDrivePID(x_pid, y_pid, heading_pid, -20, -7, 165, 5);
  // x_pid.Reset();
  // y_pid.Reset();
  // heading_pid.Reset();

  // aon::odometry::Debug();
}
void tempRoutine() {  // temporary routne to test GUI

  aon::PID x_pid = aon::PID(10, 0, 0.75);
  aon::PID y_pid = aon::PID(10, 0, 0.75);
  aon::PID heading_pid = aon::PID(0.15, 0, 0.0099);

  MoveDrivePID(x_pid, y_pid, heading_pid, 10, 0, 0, 3);
  x_pid.Reset();
  y_pid.Reset();
  heading_pid.Reset();
}

void programming_skills() {
  first_routine();
  aon::PID x_pid = aon::PID(10, 0, 0.75);
  aon::PID y_pid = aon::PID(10, 0, 0.75);
  aon::PID heading_pid = aon::PID(0.15, 0, 0.0099);

  MoveDrivePID(x_pid, y_pid, heading_pid, -18, -10, 250, 3);

  expansion.set_value(1);
}

int first_routine_wrapper() {  // fixing gui return type Temp
  // aon::odometry::Debug();
  first_routine();
  return 0;
}

int tempRoutine_wrapper() {  // fixing gui return type Temp
  // aon::odometry::Debug();
  tempRoutine();
  return 0;
}

#endif
};  // namespace aon
