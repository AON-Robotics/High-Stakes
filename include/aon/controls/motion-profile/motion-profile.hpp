#ifndef MOTION_PROFILE_HPP__
#define MOTION_PROFILE_HPP__

#include <cmath>
#include <cfloat>
#include <algorithm>
#include "../../tools/logging.hpp"

namespace aon {

/**
 * \brief Base MotionProfile class
 *
 * \details This class implements some fundamental methods that all motion
 * profiles must follow. This includes error inverse position profiles and the
 * correction ratio logic.
 */
class MotionProfile {
 protected:
  /// \brief Distance to be travelled (in appropriate distance units)
  double D = 0.;
  /// \brief Time that the entire motion will take (in appropriate time units).
  double T = DBL_MIN;
  /// \brief Indicates how drastically we will respond to overshoots (unitless).
  double correction_ratio = 1.;

  /**
   * \brief Fundamental speed profile
   *
   * \details All motion profiles will have a different implementation for their
   * speed profiles. This speed profile must be a function of time defined
   * between 0 and T, and zero everywhere else, and its integral must match with
   * `s(t)`.
   *
   * \param t Current time instant
   * \return double Velocity at this time instant.
   */
  virtual double v(double t) = 0;

  /**
   * \brief Fundamental position profile
   *
   * \details All motion profiles will have a different implementation for their
   * speed profiles. This function is the integral of that speed profile, which
   * I decided to call position profile. It must be defined between 0 and T, 0
   * for t < 0, and D for t > T.
   *
   * \param t Current time instant
   * \return double Position at this time instant.
   */
  virtual double s(double t) = 0;

 public:
  /// \brief Getter for motion profile `D`
  /// \see aon::MotionProfile::D
  double GetD() { return D; }

  /// \brief Getter for motion profile `T`
  /// \see aon::MotionProfile::T
  double GetT() { return T; }

  /// \brief Getter for motion profile `correction_ratio`
  /// \see aon::MotionProfile::T
  double GetCorrectionRatio() { return correction_ratio; }

  /**
   * \details This is the actual speed profile function users will be able to
   * see and use. It extends the fundamental speed profile by adding the error
   * correction component to compensate for overshoots.
   *
   * \param t Current time instant
   * \return double Velocity at this time instant.
   *
   * \see aon::MotionProfile::v
   */
  double SpeedProfile(double t) {
    return v(t) + correction_ratio * (v(-t) - v(2.0 * T - t));
  }

  /**
   * \details This is the actual position profile function users will be able to
   * see and use. It extends the fundamental position profile so it is possible
   * to add the error correction component to compensate for overshoots.
   *
   * \param t Current time instant
   * \return double Velocity at this time instant.
   *
   * \see aon::MotionProfile::s
   */
  double PositionProfile(double t) {
    return D + s(t) - (s(-t) + s(2.0 * T - t));
  };

  /**
   * \brief Numerically calculates the inverse of the position profile
   *
   * \note Uses binary search to look for roots of s(t) = d => s(t) - d = 0.
   * I tried false-position, ridder's method, and other numerical algorithms
   * before, but found out that this method is less error-prone, easier to
   * debug, and so basic it runs even faster than the others.
   *
   * \param d Position at this instant of time
   * \return double Time this position corresponds to
   */
  double InversePositionProfile(double d) {
    double t_left = -1.5 * T, t_right = 2.5 * T,
           t_mid = t_left + (t_right - t_left) / 2.0;

    double s_left = PositionProfile(t_left) - d,
           s_right = PositionProfile(t_right) - d,
           s_mid = PositionProfile(t_mid) - d;

    for (int idx = 0; idx < 20; idx++) {
      if (s_mid == 0) {
        break;
      } else if (std::signbit(s_left) != std::signbit(s_mid)) {
        t_right = t_mid;
        s_right = s_mid;
      } else {
        t_left = t_mid;
        s_left = s_mid;
      }
      t_mid = t_left + (t_right - t_left) / 2.0;
      s_mid = PositionProfile(t_mid) - d;
    }

    if (isnormal(t_mid)) return t_mid;
    return this->GetT() / this->GetD() * d;
  }
};

};  // namespace aon

#include "./trapezoid-profile.hpp"
#include "./exponential-profile.hpp"
#include "./quintic-profile.hpp"
#include "./hold-motion-profile.hpp"

#endif  // MOTION_PROFILE_HPP_
