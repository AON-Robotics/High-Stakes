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
      // aon::testGPSNew(0, 0);
      // double angle = aon::calculateTurnDeg(aon::Vector().SetPosition(0, 0), aon::Vector().SetPosition(gps.get_status().x, gps.get_status().y));

      // if(angle < 0){
      //   piston.set_value(aon::toggle(piston_on));
      // }
      // else if(angle < 90) {
      //   aon::testEndpoint();
      // }
      // else if(angle < 180)
      // {
      //   aon::testEndpoint(-100);
      // }
      // else if(angle < 270){
      //   aon::testEndpoint(200);
      // }
      // else if(angle < 360){
      //   aon::testEndpoint(-200);
      // }


      // aon::teamRingsRoutine();


      // aon::odometry::Update();
      pros::delay(10);
      //aon::odometry::Debug();
    }
  }
