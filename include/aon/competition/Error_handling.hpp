#pragma once
#include <cmath>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <vector>
#include "../constants.hpp"
#include "../globals.hpp"
#include "../sensing/odometry.hpp"
#include "../controls/pid/pid.hpp"
#include "../controls/holonomic-motion.hpp"
#include "pros/rtos.hpp"

namespace aon {
namespace error_handling {

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// NOTE: testMotors() works well but Vex Motors have in-built current limiting and thermal protection   //
//       circuitry that will automatically prevent damage regardless of the software                    //
//       monitoring provided by this module, can be used if motors aren't mechanically linked by gears. //
//////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Determines if a motor status flag indicates a healthy motor
 * 
 * Only specific flag values (0x0, 0x6, 0x16, 0x26) are considered healthy.
 * These values correspond to normal operational states of VEX motors.
 * 
 * @param flag The motor status flag to check
 * @return true if the flag indicates a healthy motor, false otherwise
 */
inline bool isHealthyFlag(int flag) {
    return flag == 0x0 ||
           flag == 0x6 ||
           flag == 0x16 ||
           flag == 0x26;
}

/**
 * @brief Outputs a debug message to the console
 * 
 * Prefixes all messages with "[ERROR_HANDLER]" for easy identification
 * and adds a small delay to ensure message visibility.
 * 
 * @param msg The message to output
 */
inline void debugCout(const std::string &msg) {
    std::cout << "[ERROR_HANDLER] " << msg << std::endl;
    pros::delay(100);
}

/**
 * @brief Tests all drive motors and updates the disabled motors map
 * 
 * Checks left/right motor pairs for unhealthy status flags. When an
 * unhealthy motor is detected, both motors in the pair are disabled
 * to maintain drive balance. The function updates the global disabledMotors
 * map with the current motor health status.
 */
inline void testMotors() {
    static const std::vector<std::pair<okapi::Motor*, okapi::Motor*>> motorPairs = {
        { &leftMotors[0], &rightMotors[0] },
        { &leftMotors[1], &rightMotors[1] },
        { &leftMotors[2], &rightMotors[2] }
    };
    
    // Inspect each pair
    for (auto &pr : motorPairs) {
        okapi::Motor* mA = pr.first;
        okapi::Motor* mB = pr.second;
        bool okA = isHealthyFlag(mA->getFlags());
        bool okB = isHealthyFlag(mB->getFlags());
        
        if (!okA || !okB) {
            disabledMotors[mA->getPort()] = true;
            disabledMotors[mB->getPort()] = true;
            debugCout(
                "Disabling ports "
                + std::to_string(mA->getPort())
                + " & "
                + std::to_string(mB->getPort())
            );
        } else {
            disabledMotors.erase(mA->getPort());
            disabledMotors.erase(mB->getPort());
        }
    }
    
    // Dump the map for visibility
    debugCout("---- Disabled Motors Map ----");
    if (disabledMotors.empty()) {
        debugCout(" <none>");
    } else {
        for (auto &entry : disabledMotors) {
            debugCout(
                " Port " + std::to_string(entry.first)
                + " => " + (entry.second ? "DISABLED" : "ENABLED")
            );
        }
    }
    debugCout("-----------------------------");
}

/**
 * @brief Background task function that periodically tests motors
 * 
 * This function runs in a separate task and calls testMotors() approximately
 * 25 times per second (every 40ms), providing continuous motor health monitoring
 * without blocking the main control flow.
 * 
 * @param void* Unused parameter required by PROS task API
 */
inline void ErrorHandlerTask(void*) {
    debugCout("ErrorHandlerTask started");
    while (true) {
        testMotors();
        pros::delay(40);
    }
}

/**
 * @brief Starts the error handling background task
 * 
 * Creates a new PROS task running the ErrorHandlerTask function.
 * This function should be called once at program initialization
 * to begin continuous motor monitoring.
 */
inline void StartErrorHandlingTask() {
    static pros::Task t(ErrorHandlerTask, nullptr, "ErrorHandlerTask");
}

} // namespace error_handling
} // namespace aon