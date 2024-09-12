#include "main.hpp"

void initialize() {
  pros::delay(100);
  aon::logging::Initialize();
  aon::ConfigureMotors();
  // aon::odometry::Initialize();
  pros::lcd::initialize();
  pros::delay(1000);
  std::string list_of_ports = "";
  //opcontrol();
//   for (int i = 1; i <= 21; i += 1) {
//     bool portinfo = aon::port::CheckPort(i);
//     if (portinfo == false)
//       list_of_ports = list_of_ports + std::to_string(i) + ",";
//   }
//   pros::lcd::print(0, "Ports %s are defective", list_of_ports);
//   std::cerr << "Ports " << list_of_ports;
}

void disabled() {}

void competition_initialize() {}

void autonomous() {

  #if USING_15_INCH_ROBOT
    //aon::blockade_routine();
    //aon::run_forward();
  #else

  #endif
}

void opcontrol() {

  while (true) {
    //aon::autonomous_movement_test();
    //aon::motiontest();
    //aon::programming_skills();
    aon::modular_autonomous::start_autonomous();
    // aon::operator_control::Run(aon::operator_control::kManes);
    // aon::odometry::Debug();

    pros::delay(10);
  }
}
