#ifndef AON_SENSING_ODOMETRY_HPP_
#define AON_SENSING_ODOMETRY_HPP_

#include <cmath>
#include "../constants.hpp"
#include "../../api.h"
#if GYRO_ENABLED
#include "../../okapi/api.hpp"
#endif
#include "../tools/vector.hpp"
#include "../globals.hpp"

/**
 * \namespace aon::odometry
 *
 * \brief Odometry namespace to try to scope such generic functions as `GetX()`,
 * `GetY()` and `GetDegrees()`
 *
 * \par Requisites:
 *    1. Declare the following global constants in `constants.hpp`:
 *       - TRACKING_WHEEL_DIAMETER (measure this)
 *       - DEGREES_PER_REVOLUTION    (= 360)
 *       - INITIAL_ODOMETRY_X ( = 0)
 *       - INITIAL_ODOMETRY_Y ( = 0)
 *       - INITIAL_ODOMETRY_THETA ( = 0)
 *       - DISTANCE_LEFT_TRACKING_WHEEL_CENTER (measure this)
 *       - DISTANCE_RIGHT_TRACKING_WHEEL_CENTER (measure this)
 *       - DISTANCE_BACK_TRACKING_WHEEL_CENTER (measure this)
 *       - GYRO_ENABLED ( = true | = false)
 *       - GYRO_CONFIDENCE ( = 0.25)
 *       - GYRO_FILTER_LENGTH ( = 5)
 *    2. Have available the `vector.hpp` header file
 *    3. Have available pros and okapilib
 *    4. Have available the `globals.hpp` header file with `encoder_left`,
 * `encoder_right`, and `encoder_back` objects instantiated and `gyroscope` if
 * the GYRO_ENABLED is true.
 *
 *  \par Instructions
 *    1. Call the `Initialize` function
 *    2. Call the `Update` function as frequently as possible to calculate the
 * pose.
 *
 * */
namespace aon::odometry {

// ============================================================================
//   __   __        _      _    _
//   \ \ / /_ _ _ _(_)__ _| |__| |___ ___
//    \ V / _` | '_| / _` | '_ \ / -_|_-<
//     \_/\__,_|_| |_\__,_|_.__/_\___/__/
//
// ============================================================================

#if GYRO_ENABLED
//> Gyro median filter
okapi::MedianFilter<GYRO_FILTER_LENGTH> gyro_filter;
//> Helps implement the complementary filter fusing the gyro and encoder data
double prev_gyro;
#endif

/**
 * \struct STRUCT_encoder
 *
 * \brief Helps store encoder data from current and previous odometry
 * iterations.
 * */
struct STRUCT_encoder {
  //> Current value in \b degrees
  double current_value;
  //> Previous value in \b degrees
  double previous_value;
  //> previous_value - current_value
  double delta;
  //> Calculated distance that tracking wheel has traveled
  double current_distance;
  //> Previous calculated distance that tracking wheel has traveled
  double previous_distance;
  //> previous_distance - current_distance
  double delta_distance;
};

//> Intermediate variable that helps to calculate absolute orientation
double theta_m;
//> Calculate delta angle each iteration using tracking wheel data or gyro
double delta_theta;
//> Stores the change in position in local reference plane
Vector delta_d_local;
//> Final calculated orientation in both \b radians and \b degrees
Angle orientation;
//> Final calculated position in \b inches
Vector position;

//> Mutex for x position to prevent race condition when retrieving value
pros::Mutex x_mutex;
//> Mutex for y position to prevent race condition when retrieving value
pros::Mutex y_mutex;
//> Mutex for orientation to prevent race condition when retrieving value
pros::Mutex orientation_mutex;

//> Encoder left struct instance
STRUCT_encoder encoder_left_data;
//> Encoder right struct instance
STRUCT_encoder encoder_right_data;
//> Encoder back struct instance
STRUCT_encoder encoder_back_data;

// ============================================================================
//     ___     _   _                __       ___      _   _
//    / __|___| |_| |_ ___ _ _ ___ / _|___  / __| ___| |_| |_ ___ _ _ ___
//   | (_ / -_)  _|  _/ -_) '_(_-< > _|_ _| \__ \/ -_)  _|  _/ -_) '_(_-<
//    \___\___|\__|\__\___|_| /__/ \_____|  |___/\___|\__|\__\___|_| /__/
//
// ============================================================================

//> Getter for X variable (unit of \b inches)
inline double GetX() {
  x_mutex.take(1);
  const double kTemp = position.GetX();
  x_mutex.give();
  return kTemp;
}
//> Setter for X variable (unit of \b inches)
inline void SetX(const double value) {
  x_mutex.take(1);
  position.SetX(value);
  x_mutex.give();
}

//> Getter for Y variable (unit of \b inches)
inline double GetY() {
  y_mutex.take(1);
  const double kTemp = position.GetY();
  y_mutex.give();
  return kTemp;
}
//> Setter for Y variable (unit of \b inches)
inline void SetY(const double value) {
  y_mutex.take(1);
  position.SetY(value);
  y_mutex.give();
}

//> Special getter function for vector
inline Vector GetPosition() { return Vector().SetPosition(GetX(), GetY()); }
//> Shorthand for odometry::GetPosition()
inline Vector GetPos() { return GetPosition(); }

/**
 * \brief Get current pose's angle in \b degrees
 *
 * \returns Returns current pose's angle in \b degrees
 */
inline double GetDegrees() {
  orientation_mutex.take(1);
  const double kDegrees = orientation.GetDegrees();
  orientation_mutex.give();
  return kDegrees;
}

/**
 * \brief Set current pose's angle in \b degrees
 *
 * \param degrees Input value to set the current angle to
 */
inline void SetDegrees(const double degrees) {
  orientation_mutex.take(1);
  orientation.SetDegrees(degrees);
  orientation_mutex.give();
  delta_theta = 0.0;
}

/**
 * \brief Get current pose's angle in \b radians
 *
 * \returns Returns current pose's angle in \b radians
 */
inline double GetRadians() {
  orientation_mutex.take(1);
  const double kRadians = orientation.GetRadians();
  orientation_mutex.give();
  return kRadians;
}

/**
 * \brief Set current pose's angle in \b radians
 *
 * \param radians Input value to set the current angle to
 *
 * \warning Sets angles in units of \b radians. INPUT MUST BE IN \b RADIANS
 * */
inline void SetRadians(const double radians) {
  orientation_mutex.take(1);
  orientation.SetRadians(radians);
  orientation_mutex.give();
  delta_theta = 0.0;
}

// ============================================================================
//    __  __      _        ___             _   _
//   |  \/  |__ _(_)_ _   | __|  _ _ _  __| |_(_)___ _ _  ___
//   | |\/| / _` | | ' \  | _| || | ' \/ _|  _| / _ \ ' \(_-<
//   |_|  |_\__,_|_|_||_| |_| \_,_|_||_\__|\__|_\___/_||_/__/
//
// ============================================================================

/**
 * \brief resets Odometry values using the particular parameters
 *
 * \param x X position in \b inches
 * \param y Y position in \b inches
 * \param theta Angular position in \b degrees
 *
 * \note Blocks for 2 seconds when gyro is enabled or a couple of milliseconds
 * when using encoders.
 */
inline void ResetCurrent(const double x, const double y, const double theta) {
  const double kCurrentL = encoder_left.get_position() / 100.0;
  const double kCurrentR = encoder_right.get_position() / 100.0;
  const double kCurrentB = encoder_back.get_position() / 100.0;
  const double kConversionFactor =
      M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION;

  // Reset encoder's struct variables
  encoder_left_data = {kCurrentL,
                       kCurrentL,
                       0.0,
                       kCurrentL * kConversionFactor,
                       kCurrentL * kConversionFactor,
                       0.0};

  encoder_right_data = {kCurrentR,
                        kCurrentR,
                        0,
                        kCurrentR * kConversionFactor,
                        kCurrentR * kConversionFactor,
                        0.0};

  encoder_back_data = {kCurrentB,
                       kCurrentB,
                       0,
                       kCurrentB * kConversionFactor,
                       kCurrentB * kConversionFactor,
                       0.0};

  // Preset odometry values
  theta_m = 0.0;
  delta_theta = 0.0;
  delta_d_local.SetPosition(0.0, 0.0);

  SetDegrees(theta);
  SetX(x);
  SetY(y);

#if GYRO_ENABLED
  gyroscope.reset();
  pros::delay(3000);
#endif
}

//> Resets the Odometry values with `INITIAL_ODOMETRY_X`,Y and T constants.
inline void ResetInitial() {
  ResetCurrent(INITIAL_ODOMETRY_X, INITIAL_ODOMETRY_Y, INITIAL_ODOMETRY_THETA);
}

/**
 * \brief Initialization function which also callibrates gyro
 *
 * \details Takes up to 2 seconds if gyro is also being used.
 */
inline void Initialize() {
#if GYRO_ENABLED
  prev_gyro = 0.0;
#endif
  encoder_left.reset();
  encoder_right.reset();
  encoder_back.reset();

  encoder_left.set_position(0);
  encoder_right.set_position(0);
  encoder_back.set_position(0);

  encoder_left.reset_position();
  encoder_right.reset_position();
  encoder_back.reset_position();

  ResetInitial();
}

/**
 * \brief Fundamental function for Odometry.
 *
 * \details Uses changes in encoder and gyroscope to update current pose
 * */

inline void Update() {
  const double kCurrentL = encoder_left.get_position() / 100.0;
  const double kCurrentR = encoder_right.get_position() / 100.0;
  const double kCurrentB = encoder_back.get_position() / 100.0;
  const double kConversionFactor =
      M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION;

  // Update difference between readings
  encoder_left_data.delta = kCurrentL - encoder_left_data.previous_value;
  encoder_right_data.delta = kCurrentR - encoder_right_data.previous_value;
  encoder_back_data.delta = kCurrentB - encoder_back_data.previous_value;

  encoder_left_data.delta_distance =
      kConversionFactor * encoder_left_data.delta;
  encoder_right_data.delta_distance =
      kConversionFactor * encoder_right_data.delta;
  encoder_back_data.delta_distance =
      kConversionFactor * encoder_back_data.delta;

  // "Shift back" the encoder readings ...
  encoder_left_data.previous_value = encoder_left_data.current_value;
  encoder_right_data.previous_value = encoder_right_data.current_value;
  encoder_back_data.previous_value = encoder_back_data.current_value;

  encoder_left_data.previous_distance = encoder_left_data.current_distance;
  encoder_right_data.previous_distance = encoder_right_data.current_distance;
  encoder_back_data.previous_distance = encoder_back_data.current_distance;

  // so they can now be updated
  encoder_left_data.current_value = kCurrentL;
  encoder_right_data.current_value = kCurrentR;
  encoder_back_data.current_value = kCurrentB;

  encoder_left_data.current_distance =
      kConversionFactor * encoder_left_data.current_value;
  encoder_right_data.current_distance =
      kConversionFactor * encoder_right_data.current_value;
  encoder_back_data.current_distance =
      kConversionFactor * encoder_back_data.current_value;

  // BEGIN WITH ODOMETRY CALCULATIONS
  // 🚨⚠ WARNING ⚠🚨: ALL ANGLES ARE IN RADIANS

  delta_theta =
      (encoder_left_data.delta_distance - encoder_right_data.delta_distance) /
      (DISTANCE_LEFT_TRACKING_WHEEL_CENTER +
       DISTANCE_RIGHT_TRACKING_WHEEL_CENTER);

// Complementary filter
#if GYRO_ENABLED
  gyro_filter.filter(gyroscope.get_rotation());
  const double kDeltaGyro =
      (gyro_filter.getOutput() - prev_gyro) * M_PI / 360.0;
  delta_theta = (1.0 - GYRO_CONFIDENCE) * delta_theta +
                GYRO_CONFIDENCE * 4.0 * kDeltaGyro;
  prev_gyro = gyro_filter.getOutput();
#endif

  if (delta_theta) {
    const double kRR = encoder_right_data.delta_distance / delta_theta +
                       DISTANCE_RIGHT_TRACKING_WHEEL_CENTER;
    const double kRS = encoder_back_data.delta_distance / delta_theta +
                       DISTANCE_BACK_TRACKING_WHEEL_CENTER;

    delta_d_local.SetY(2.0 * std::sin(delta_theta / 2.0) * kRR);
    delta_d_local.SetX(2.0 * std::sin(delta_theta / 2.0) * kRS);

  } else {
    delta_d_local.SetX(encoder_back_data.delta_distance);
    delta_d_local.SetY(encoder_right_data.delta_distance);
    delta_theta = 0.0;
  }

  theta_m = GetRadians() + delta_theta / 2.0;

  SetY(GetY() + delta_d_local.GetY() * std::cos(theta_m) / 2.0);
  SetY(GetY() - delta_d_local.GetX() * std::sin(theta_m) / 2.0);

  SetX(GetX() + delta_d_local.GetY() * std::sin(theta_m) / 2.0);
  SetX(GetX() + delta_d_local.GetX() * std::cos(theta_m) / 2.0);

  SetRadians(GetRadians() + delta_theta / 2.0);
}

// ============================================================================
//    _____       _
//   |_   _|__ __| |_ ___
//     | |/ -_|_-<  _(_-<
//     |_|\___/__/\__/__/
//
// ============================================================================

/**
 * \brief Simple debug function that prints odometry values
 *
 * \details Blocking function that helps check if there are any issues with
 * odometry
 *
 * \note Requires initialize pros::lcd and calling the odometry::Initialize
 *       function
 * */
inline void Debug() {
  while (true) {
    pros::lcd::print(0, "X: %0.3f", GetX());
    pros::lcd::print(1, "Y: %0.3f", GetY());
    pros::lcd::print(2, "T: %0.3f", GetDegrees());

#if GYRO_ENABLED
    pros::lcd::print(3, "Gyro Angle: %0.3f", gyro_filter.getOutput());
#endif

    pros::lcd::print(4, "Enc L: %.2f degrees",
                     encoder_left.get_position() / 100.0);
    pros::lcd::print(5, "Enc R: %.2f degrees",
                     encoder_right.get_position() / 100.0);
    pros::lcd::print(6, "Enc B: %.2f degrees",
                     encoder_back.get_position() / 100.0);

    odometry::Update();
    pros::delay(10);
  }
}

}  // namespace aon::odometry

#endif  // AON_SENSING_ODOMETRY_HPP_
