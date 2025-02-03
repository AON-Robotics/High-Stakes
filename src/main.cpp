#include "main.hpp"
#include "../include/aon/competition/state_machine.hpp"
#include "../include/aon/competition/intake_engine.hpp"

void initialize() {
  aon::logging::Initialize();
  aon::ConfigureMotors();
  aon::ConfigureColors();
  aon::odometry::Initialize();
  pros::Task gui_task(aon::gui::Initialize);
}

void disabled() {}

void competition_initialize() {}

void autonomous() { 
 
  // aon::AutonomousReader->ExecuteFunction("autonomous");
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
void opcontrol() {
  while (true) {
    aon::operator_control::Run(aon::operator_control::kManes);
    pros::delay(10);
  }
}
