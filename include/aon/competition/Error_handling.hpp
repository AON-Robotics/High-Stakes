#pragma once

#include <cmath>
#include <algorithm>
#include "../constants.hpp"
#include "../globals.hpp"
#include "../sensing/odometry.hpp"
#include "../controls/pid/pid.hpp"
#include "../controls/holonomic-motion.hpp"

namespace aon {
namespace error_handling {

inline void testMotors() {
  std::vector<std::pair<okapi::Motor*, okapi::Motor*>> motorPairs = {
      {&Left1, &Right1}, {&Left2, &Right2}, {&Left3, &Right3}
  };

  for (const auto &pair : motorPairs) {
    okapi::Motor* primaryMotor = pair.first;
    okapi::Motor* secondaryMotor = pair.second;

    if (primaryMotor->getFlags() == 0x01) { 
      secondaryMotor->moveVelocity(0);
    }
    if (secondaryMotor->getFlags() == 0x01) {
      primaryMotor->moveVelocity(0);
    }
  }
}

inline void ErrorHandlerTask(void *param) {  
    while (true) {
    testMotors();
    pros::delay(5);
  }
}

inline void StartErrorHandlingTask() {
  static pros::Task errorTask(ErrorHandlerTask, nullptr, "ErrorHandlerTask");
    }
  }
}
