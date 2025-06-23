/**
 * @file s-curve-profile.hpp
 * @author Kevin Gomez
 * @brief S-Curve motion profiling class for smooth and precise robot movements based on odometry
 * @version 1.0
 * @date 2025-06-19
 */

#pragma once

#include <cmath>
#include "constants.hpp"
#include <globals.hpp>

class MotionProfile {

    double MAX_VELOCITY, MAX_ACCELERATION, MAX_DECELERATION, JERK;
    double currVelocity = 0;
    double currAccel = 0;

    public:

        MotionProfile(double MAX_VELOCITY, double MAX_ACCELERATION, double MAX_DECELERATION, double JERK) {
            this->MAX_VELOCITY = std::abs(MAX_VELOCITY); // RPM
            this->MAX_ACCELERATION = std::abs(MAX_ACCELERATION); // RPM/s
            this->MAX_DECELERATION = std::abs(MAX_DECELERATION); 
            this->JERK = std::abs(JERK); // RPM/(s^2)
        }

        /// @brief The core of the class functionality: calculates the current velocity to send to the motors to get an optimal acceleration and deceleration curve for smooth and precise movements
        /// @param remainingDist The remaining distance in \b inches towards the objective
        /// @param dt The time that passed since the last funcion call in \b seconds
        /// @return The updated value for the velocity in \b RPM
        double update(const double &remainingDist, const double &dt = 0.02) {

            // Acceleration
            // For the condition, consider half the deceleration for accuracy (there is an error of half an inch almost constant when not used, I have to investigate a bit further on that part but it works fine like this)
            if(remainingDist <= getSpeed(this->currVelocity) * getSpeed(this->currVelocity) / (2 * getSpeed(this->MAX_DECELERATION * .5))){
                this->currAccel = - this->MAX_DECELERATION;
            }
            // Constant velocity
            else if (this->currVelocity == this->MAX_VELOCITY) {
                this->currAccel = 0;
            }
            // Deceleration
            else {
                this->currAccel = std::min(currAccel + (this->JERK * dt), this->MAX_ACCELERATION);
            }

            this->currVelocity += this->currAccel * dt;
            this->currVelocity = std::min(this->currVelocity,  this->MAX_VELOCITY);
            return this->currVelocity;
        }

        /// @brief Resets the velocity and acceleraton for reusability
        void reset(){
            this->currVelocity = 0;
            this->currAccel = 0;
        }

        /// @brief Sets the velocity in case profile is not started from rest
        /// @param velocity The current velocity of the robot
        void setVelocity(const double &velocity = 0) {
            this->currVelocity = velocity;
        }

        /// @brief Sets the acceleration in case profile is not started from rest
        /// @param accel The current acceleration of the robot
        void setAccel(const double &accel = 0) {
            this->currAccel = accel;
        }
};

/// @brief Determines the speed of the robot given drivetrain motors' RPM
/// @param RPM The RPM for which to calculate the velocity (default current RPM)
/// @return The speed in \b in/s at which the robot would move at the given RPM
/// @note Test the accuracy precision of the `getActualVelocity()` method,
/// @note it may be possible to need to use `get_velocity()` from `pros::Rotation` which uses \b centidegrees.
/// @note The distance units depend on the units used for measuring `DRIVE_WHEEL_DIAMETER`.
inline double getSpeed(const double &RPM = (int)driveFull.getActualVelocity()){
  double circumference = DRIVE_WHEEL_DIAMETER * M_PI;
  double RPS = RPM / 60;
  double speed = MOTOR_TO_DRIVE_RATIO * circumference * RPS;
  return speed;
}