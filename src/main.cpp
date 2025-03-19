#include "../include/main.hpp"
#include "../include/aon/competition/state_machine.hpp"
#include "../include/aon/competition/intake_engine.hpp"
#include "aon/competition/autonomous-routines.hpp"
#include "../include/aon/competition/Error_handling.hpp"
#include "aon/globals.hpp"

void initialize() {
  aon::logging::Initialize();
  pros::lcd::initialize();
  aon::ConfigureMotors();
  aon::ConfigureColors();
  aon::odometry::Initialize();
  pros::Task gui_task(aon::gui::Initialize);
  aon::operator_control::Run(aon::operator_control::kManes);
  aon::error_handling::StartErrorHandlingTask();
  // aon::error_handling::testMotors();
}

void disabled() {}

void competition_initialize() {}

void autonomous() { 
 }

// During development
// Program slot 1 with Pizza Icon is for opcontrol
// Program slot 2 with Planet Icon is for autonomous routine
void opcontrol() {
  while (true) {
    aon::operator_control::Run(aon::operator_control::kManes);
    pros::delay(10);
  }
}
