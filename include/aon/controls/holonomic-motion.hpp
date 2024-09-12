/**
 * \file holonomic_motion.hpp
 *
 * \brief Implementation of holonomic motion and some control algorithms that
 * make use of it.
 * */
#pragma once

#include <cmath>
#include <algorithm>
#include "../constants.hpp"
#include "./motion-profile/motion-profile.hpp"
#include "../sensing/odometry.hpp"
#include "../globals.hpp"

namespace aon::holonomic_motion {

void MoveHolonomicMotion(double vx, double vy, double vT,
                         bool use_odom = true) {
  struct Components {
    double theta;
    double x;
    double y;
  };

  auto h = [](double beta, double gamma, double phi, double x, double y,
              double r) {
    Components output;

    if (r == 0.0) return output;
    if (std::cos(gamma) == 0.0) return output;

    output.theta = x * std::sin(beta + gamma) - y * std::cos(beta + gamma);
    output.theta /= r * std::cos(gamma);

    output.x = std::cos(beta + gamma + phi);
    output.x /= r * std::cos(gamma);

    output.y = std::sin(beta + gamma + phi);
    output.y /= r * std::cos(gamma);

    return output;
  };

  const double r = std::abs(DRIVE_WHEEL_DIAMETER / 2.0);
  vT = M_PI * vT / 180.0;

  // Odometry uses CW as positive while conventionally CCW is positive
  const double phi = (use_odom) ? -aon::odometry::GetRadians() : 0.0;

  // Compute velocity for each of the wheels (inches per second)
  Components h1 = h(M_PI_4, 0.0, phi, -HOLONOMIC_MOTION_X_DISTANCE,
                    HOLONOMIC_MOTION_Y_DISTANCE, r);
  const double u1 = h1.theta * vT + h1.x * vx + h1.y * vy;

  Components h2 = h(3.0 * M_PI_4, 0.0, phi, -HOLONOMIC_MOTION_X_DISTANCE,
                    -HOLONOMIC_MOTION_Y_DISTANCE, r);
  const double u2 = h2.theta * vT + h2.x * vx + h2.y * vy;

  Components h3 = h(-M_PI_4, 0.0, phi, HOLONOMIC_MOTION_X_DISTANCE,
                    HOLONOMIC_MOTION_Y_DISTANCE, r);
  const double u3 = h3.theta * vT + h3.x * vx + h3.y * vy;

  Components h4 = h(-3.0 * M_PI_4, 0.0, phi, HOLONOMIC_MOTION_X_DISTANCE,
                    -HOLONOMIC_MOTION_Y_DISTANCE, r);
  const double u4 = h4.theta * vT + h4.x * vx + h4.y * vy;

  // Compute conversion factor in order to get RPMs.
  // 60 secs = 1 min, 2 * PI rad = 1 rev
  // rad per s * (1 rev / (2 * PI rad)) * (60 s / 1 min) = rev per min
  const double RPS2RPM = 30.0 / M_PI;

  // Move motors
  left_side.moveVelocity(u1 * RPS2RPM);
  right_side.moveVelocity(-u2 * RPS2RPM);
  /*drive_front_right.moveVelocity(-u3 * RPS2RPM);
  drive_back_right.moveVelocity(-u4 * RPS2RPM);*/
}

inline void emptyFunction(int t) {}

void Move(aon::MotionProfile& x_profile, aon::MotionProfile& y_profile,
          aon::MotionProfile& theta_profile, double timeout,
          std::function<void(int)> function = emptyFunction) {
  const uint64_t start_time = pros::micros();
  const double start_x = aon::odometry::GetX();
  const double start_y = aon::odometry::GetY();
  const double start_theta = -aon::odometry::GetDegrees();

  while (pros::micros() - start_time < timeout * 1E6) {
    aon::odometry::Update();

    const double t_x =
        x_profile.InversePositionProfile(aon::odometry::GetX() - start_x);
    const double t_y =
        y_profile.InversePositionProfile(aon::odometry::GetY() - start_y);
    const double t_theta = theta_profile.InversePositionProfile(
        -aon::odometry::GetDegrees() - start_theta);

    const double vx = x_profile.SpeedProfile(t_x);
    const double vy = y_profile.SpeedProfile(t_y);
    const double vT = theta_profile.SpeedProfile(t_theta);

    aon::holonomic_motion::MoveHolonomicMotion(vx, vy, vT);

    function((pros::micros() - start_time) / 1E3);

    pros::delay(10);
  }

  aon::holonomic_motion::MoveHolonomicMotion(0, 0, 0);
}

};  // namespace aon::holonomic_motion