#include "main.hpp"
#include "../include/aon/competition/state_machine.hpp"
#include "../include/aon/competition/intake_engine.hpp"

void initialize() {
  // aon::logging::Initialize();
  pros::lcd::initialize();
  aon::ConfigureMotors();
  aon::ConfigureColors();
  aon::odometry::Initialize();
  // pros::Task gui_task(aon::gui::Initialize);
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
    #if TESTING_AUTONOMOUS
    // aon::move(12);
    // aon::testMotionProfile(60 * 2);
    // pros::delay(30000);
    //aon::FollowWithTurret();
    //aon::AlignRobotToStake();
    //aon::turretFollow();
    //aon::alignRobotToDisk();
    aon::continuous_scan();
    
    #else
    aon::operator_control::Run(aon::operator_control::kManes);
    #endif
    pros::delay(10);
  }
}
