#include "main.hpp"
#include "../include/aon/competition/state_machine.hpp"
#include "../include/aon/competition/intake_engine.hpp"

void initialize() {
  // aon::logging::Initialize();
  pros::lcd::initialize();
  aon::ConfigureMotors(false);
  aon::ConfigureColors();
  aon::odometry::Initialize();
  // pros::Task gui_task(aon::gui::Initialize);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {

  if(COLOR == RED){
    aon::RedRingsRoutine();
  }
  else {
    aon::BlueRingsRoutine();
  }
  pros::delay(10);

 }

// During development
// Program slot 1 with Pizza Icon is for opcontrol
// Program slot 2 with Planet Icon is for autonomous routine
// Program slot 3 with Alien Icon is for tests or miscellaneous components
void opcontrol() {
  aon::ConfigureMotors();
  while (true) {
    #if TESTING_AUTONOMOUS
    aon::ConfigureMotors(false); // Set drivetrain to hold for auton testing

    if(COLOR == RED){
      aon::RedRingsRoutine();
    }
    else {
      aon::BlueRingsRoutine();
    }

    // aon::testIndexer();
    aon::odomTest();
    #else
    aon::operator_control::Run(aon::operator_control::kManes);
    #endif
    pros::delay(10);
  }
}
