#include "main.hpp"
#include "../include/aon/competition/state_machine.hpp"
#include "../include/aon/competition/intake_engine.hpp"

void initialize() {
  aon::logging::Initialize();
  aon::ConfigureMotors();
  aon::ConfigureColors();
  aon::odometry::Initialize();
  pros::Task gui_task(aon::gui::Initialize);
  // pros::Task intake_task(rail_state_machine);
}

void disabled() {}

void competition_initialize() {}

void autonomous() { 
 
  // while (true) {
    aon::AutonomousReader->ExecuteFunction("autonomous");
    // aon::programming_skills();
    // aon::operator_control::Run(aon::operator_control::kManes);
    // aon::odometry::Update();
    pros::delay(10);
    //aon::odometry::Debug();
    // }

 }

void opcontrol() {

    while (true) {
      // aon::AutonomousReader->ExecuteFunction("autonomous");
      aon::operator_control::Run(aon::operator_control::kManes);

      // aon::driveIntoTaurus(BLUE);
      // aon::testGPS(0, 0);


      // aon::teamRingsRoutine();


      // aon::odometry::Update();
      pros::delay(10);
      //aon::odometry::Debug();
    }
  }
