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

#if USING_15_INCH_ROBOT

void MoveDrivePID(aon::PID pid, aon::Vector targetPos, double timeLimit, double sign = 1) {

  double avg_x = 0;
  double avg_y = 0;
  
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
  aon::Vector initialPos = aon::odometry::GetPosition();

  const double start_time = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6 - start_time) //every time the variable is called it is recalculated automatically

  //Target position minus initial position
  const double targetDiplacement = (targetPos - initialPos).GetMagnitude();
  while (time < timeLimit) {
    aon::odometry::Update();

    double currentDisplacement = (aon::odometry::GetPosition() - initialPos).GetMagnitude();

    double output = pid.Output(targetDiplacement, currentDisplacement);

    pros::lcd::print(0, "%f", currentDisplacement);

    driveLeft.moveVelocity(sign * std::clamp(output, -100., 100.));
    driveRight.moveVelocity(sign * std::clamp(output, -100., 100.));

    pros::delay(10);
  }

  driveLeft.moveVelocity(0);
  driveRight.moveVelocity(0);
#undef time
}

inline void first_routine() {
  aon::PID pid = aon::PID(10, 0, 0);
  aon::Vector target = aon::Vector().SetPosition(6.0, 0);
  aon::MoveDrivePID(pid, target, 4);
  pid.Reset();

  intake.moveVelocity(90);
  pros::delay(500);
  intake.moveVelocity(0);

  target = aon::Vector().SetPosition(0, 0);
  aon::MoveDrivePID(pid, target, 3, -1);
  pid.Reset();
}

inline void programming_skills() {
  first_routine();
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
  first_routine();
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
