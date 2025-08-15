#include "main.hpp"

void initialize() {
  // pros::Task guiTask(aon::gui::Initialize);
  // aon::logging::Initialize();
  pros::lcd::initialize();
  aon::ConfigureMotors(false);
  aon::ConfigureColors();
  aon::odometry::Initialize();
  // pros::Task odomTask(aon::odometry::Odometry);
  pros::Task safetyTask(aon::autonSafety);
  pros::Task turretFollowTask(aon::turretFollow);
  pros::Task intakeTask(aon::intakeScan);
  pros::Task turretScanTask(aon::turretScan); // TODO: combine this with the follow task
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  aon::AutonomousReader->ExecuteFunction("autonomous");
  pros::delay(10);
}

// During development
// Program slot 1 with Pizza Icon is for opcontrol
// Program slot 2 with Planet Icon is for autonomous routine
// Program slot 3 with Alien Icon is for tests or miscellaneous components

/**
 * Things to do:
 *  - Test odometry
 *  - Use gps for initial position
 *  - Check if we can put a limit in the y coordinates with odometry, make sure the turret dont overdo
 * 
 *  Idea:
 *  - To park the robot at the end, calculate how much time it takes us to reach that place, and before
 *    the time is over activate a routine to go there in time
 * 
 * 
 * Things to discuss with kevin:
 *  - The back encoder i can implement it, but i can do it in another function
 *    and depending what drivetrain we have, we use the function
 *  - Idea, have a function to collect all the data, and differents update functions
 *    for diferent drivetrain
 *  - The change in Y, it calculate depending on the angle, and the robot is never
 *    moving sideways with the current drivetrain, so i just delete the back encoder
 *  - And for the turning, i actually think of that but i thought the odometry should 
 *    calculate that and not depends in an argument, but i believe it would be magnificiant
 *  
 */
void opcontrol() {
  aon::ConfigureMotors();
  while (true) {
    #if TESTING_AUTONOMOUS
    // aon::ConfigureMotors(false); // Set drivetrain to hold for auton testing

    // aon::AutonomousReader->ExecuteFunction("autonomous");
    for(int i = 0; i < 4; i++) {
      aon::move(12);
      aon::turn(90);
    }

    aon::odometry::ResetCurrent(0, 0, 0);
    aon::odometry::Debug();

    pros::delay(3000);
    #else
    aon::operator_control::Run(aon::operator_control::DEFAULT);
    #endif
    pros::delay(10);
  }
}
