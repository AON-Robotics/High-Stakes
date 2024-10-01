#include "main.hpp"

void initialize() {
  aon::logging::Initialize();
  aon::ConfigureMotors();
  aon::odometry::Initialize();
  pros::Task gui_task(aon::gui::Initialize);


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
      aon::AutonomousReader->ExecuteFunction("autonomous");
      // aon::programming_skills();
      // aon::operator_control::Run(aon::operator_control::kManes);
      // aon::odometry::Update();
      pros::delay(10);
      //aon::odometry::Debug();
    }
  }
