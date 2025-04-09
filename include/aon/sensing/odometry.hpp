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
 * \brief Odometry namespace to calculate the position we are in the field
 *
 * \par Requisites:
 *    1. Declare the following global constants in `constants.hpp`:
 *       - TRACKING_WHEEL_DIAMETER (measure this)
 *       - DISTANCE_BETWEEN_ENCODERS (measure this)
 *       - HOW_MUCH_TO_THE_FRONT (measure this)
 *       - DEGREES_PER_REVOLUTION    (= 360)
 *       - INITIAL_ODOMETRY_X ( = 0)
 *       - INITIAL_ODOMETRY_Y ( = 0)
 *       - INITIAL_ODOMETRY_THETA ( = 0)
 *       - GYRO_ENABLED ( = true | = false)
 *    2. Have available the `vector.hpp` header file
 *    3. Have available pros and okapilib
 *    4. Have available the `globals.hpp` header file with `encoderLeft`,
 *    and `encoderRight` objects instantiated and `gyroscope` if
 *    the GYRO_ENABLED is true.
 *
 *  \par Instructions
 *    1. Call the `Initialize` function
 *    2. Call the `Update` function as frequently as possible to calculate the
 *    pose.
 * */

namespace aon::odometry {

    // ============================================================================
    //   __   __        _      _    _
    //   \ \ / /_ _ _ _(_)__ _| |__| |___ ___
    //    \ V / _` | '_| / _` | '_ \ / -_|_-<
    //     \_/\__,_|_| |_\__,_|_.__/_\___/__/
    //
    // ============================================================================

    /**
     * \struct STRUCT_encoder
     *
     * \brief Store encoder data from current and previous odometry
     * iterations.
     * */
    struct STRUCT_encoder {
      double current_value;     //> Current value in degrees
      double previous_value;    //> Previous value in degrees
      double delta;             //> previous_value - current_value

      //> All this 3 variables are in inches
      double current_distance;  //> Distance that tracking wheel has traveled
      double previous_distance; //> Previous distance that tracking wheel has traveled
      double delta_distance;    //> previous_distance - current_distance
    };

    /**
     * \struct STRUCT_encoder
     *
     * \brief Store gyro data from current and previous odometry
     * iterations.
     * */
    struct STRUCT_gyro {
      double currentDegrees;
      double prevDegrees;

      double currentRadians;
      
      double deltaRadians;
      double deltaDegrees;
    };
    
    //> Calculate delta angle each iteration using tracking wheel data
    double deltaTheta;
    //> Stores the change in position in local reference plane
    Vector deltaDlocal;
    //> Final calculated orientation in both \b radians and \b degrees
    Angle orientation;
    //> Final calculated position in \b inches
    Vector position;
    
    //> Mutex for x position to prevent race condition when retrieving value
    pros::Mutex x_mutex;
    //> Mutex for y position to prevent race condition when retrieving value
    pros::Mutex y_mutex;
    //> Mutex for absolute position
    pros::Mutex p_mutex;
    //> Mutex for orientation to prevent race condition when retrieving value
    pros::Mutex orientation_mutex;
    
    //> Encoder left struct instance
    STRUCT_encoder encoderLeft_data;
    //> Encoder back struct instance
    STRUCT_encoder encoderRight_data;
    //> Gyro struct instance
    STRUCT_gyro gyro_data;

    // ============================================================================
//     ___     _   _                __       ___      _   _
//    / __|___| |_| |_ ___ _ _ ___ / _|___  / __| ___| |_| |_ ___ _ _ ___
//   | (_ / -_)  _|  _/ -_) '_(_-< > _|_ _| \__ \/ -_)  _|  _/ -_) '_(_-<
//    \___\___|\__|\__\___|_| /__/ \_____|  |___/\___|\__|\__\___|_| /__/
//
// ============================================================================

  /**
   * \brief Get current X position in \b inches
   *
   * \returns Returns current X position in \b inches
   */
  inline double GetX() {
    x_mutex.take(1);
    const double currentX = position.GetX();
    x_mutex.give();
    return currentX;
  }
  
  /**
   * \brief Get current Y position in \b inches
   *
   * \returns Returns current Y position in \b inches
   */
  inline double GetY() {
    y_mutex.take(1);
    const double currentY = position.GetY();
    y_mutex.give();
    return currentY;
  }
  
  /**
   * \brief Set Y position in \b inches
   * 
   * \param value Input value to set current Y
   */

  inline void SetPosition(double x, double y) {
    p_mutex.take(1);
    position.SetPosition(x, y);
    p_mutex.give();
  }

  /**
   * \brief Get current pose's angle in \b degrees
   *
   * \returns Returns current pose's angle in \b degrees
   */
  inline double GetDegrees() {
    orientation_mutex.take(1);
    const double currentDegrees = orientation.GetDegrees();
    orientation_mutex.give();
    return currentDegrees;
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
    deltaTheta = 0.0;
  }
  
  /**
   * \brief Get current pose's angle in \b radians
   *
   * \returns Returns current pose's angle in \b radians
   */
  inline double GetRadians() {
    orientation_mutex.take(1);
    const double currentRadian = orientation.GetRadians();
    orientation_mutex.give();
    return currentRadian;
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
    deltaTheta = 0.0;
  }

  /**
   * \brief Get a vector with the current position
   * 
   * \return Returns new vector with current position
   */
  inline Vector GetPosition() { return Vector().SetPosition(GetX(), GetY()); }
  

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
 */
inline void ResetCurrent(const double x, const double y, const double theta) {
    const double currentAngleLeft = encoderLeft.get_position() / 100.0;
    const double currentAngleRight = encoderRight.get_position() / 100.0;
    const double currentAngleGyro = gyroscope.get_heading();

    const double conversionFactor = M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION;
  
    // Reset encoder's struct variables
    encoderLeft_data = {currentAngleLeft,                       // current position in degrees
                         currentAngleLeft,                      // previuos position in degrees
                         0,                                     // delta in degrees
                         currentAngleLeft * conversionFactor,  // current position in inches 
                         currentAngleLeft * conversionFactor,  // previuos position in inches
                         0.0};                                  // delta in inches

    encoderRight_data = {currentAngleRight,                      // current position in degrees
                         currentAngleRight,                      // previuos position in degrees
                         0,                                      // delta in degrees
                         currentAngleRight * conversionFactor,  // current position in inches 
                         currentAngleRight * conversionFactor,  // previuos position in inches
                         0.0};                                   // delta in inches

    gyro_data = {currentAngleGyro,                               // current value degrees
                 currentAngleGyro,                               // prevuios value degrees
                 currentAngleGyro * (M_PI / 180.0),              // current radians
                 0.0,                                            // delta degrees
                 0.0};                                           // delta radians
    
    // Preset odometry values
    deltaTheta = 0.0;
    deltaDlocal.SetPosition(0.0, 0.0);
  
    SetDegrees(theta);
    SetPosition(x, y);
    #if GYRO_ENABLED
    gyroscope.tare();
    pros::delay(3000);
    #endif
}

//> Resets the Odometry values with `INITIAL_ODOMETRY_X`,Y and T constants.
inline void ResetInitial() {
    ResetCurrent(INITIAL_ODOMETRY_X, INITIAL_ODOMETRY_Y, INITIAL_ODOMETRY_THETA);
}

/**
 * \brief Initialization function to put everything to 0
 */
inline void Initialize() {
    encoderLeft.set_position(0);
    encoderRight.set_position(0);
    
    encoderLeft.reset();
    encoderRight.reset();

    // Set initial position with gps
    // INITIAL_ODOMETRY_X = gps.get_x_position();
    // INITIAL_ODOMETRY_Y = gps.get_y_position();
    
    ResetInitial();
}

/**
 * \brief Fundamental function for Odometry.
 *
 * \details Uses changes in encoder (left and rigth) and gyro to calculate position
 * 
 * */

 inline void Update() {
  const double conversionFactor = M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION;

  // Read encoder values, divided by 100 to convert centidegrees to degrees
  encoderLeft_data.current_value = encoderLeft.get_position() / 100.0; 
  encoderRight_data.current_value = -encoderRight.get_position() / 100.0; // weird things happen if change port to negative

  // Convert to distances
  encoderLeft_data.current_distance = encoderLeft_data.current_value * conversionFactor;
  encoderRight_data.current_distance = encoderRight_data.current_value * conversionFactor;

  // Read gyro value
  gyro_data.currentDegrees = gyroscope.get_heading();
  gyro_data.currentRadians = gyro_data.currentDegrees * (M_PI / 180);

  // Calculate deltas
  encoderLeft_data.delta = encoderLeft_data.current_value - encoderLeft_data.previous_value;
  encoderRight_data.delta = encoderRight_data.current_value - encoderRight_data.previous_value;
  
  encoderLeft_data.delta_distance = encoderLeft_data.current_distance - encoderLeft_data.previous_distance;
  encoderRight_data.delta_distance = encoderRight_data.current_distance - encoderRight_data.previous_distance;

  gyro_data.deltaDegrees = gyro_data.currentDegrees - gyro_data.prevDegrees;
  gyro_data.deltaRadians = gyro_data.deltaDegrees * (M_PI / 180.0);
  
  // Calculate heading change using encoder angles to estimate angle better (test it later)
  double deltaA = (encoderRight_data.delta - encoderLeft_data.delta) / (DISTANCE_BETWEEN_ENCODERS + (2 * HOW_MUCH_TO_THE_FRONT / DISTANCE_BETWEEN_ENCODERS));

  // Combine gyro and encoder heading (change this later with Kalman filter, giving more priority to gyro or something), test this later
  // deltaTheta = (gyro_data.deltaRadians + deltaA) / 2.0;
  deltaTheta = gyro_data.deltaRadians;
  
  // If we are rotating
  if (deltaTheta > 0.1) {
    // Calculate the radius of rotation for each wheel
    double radiusRight = encoderRight_data.delta_distance / deltaTheta + DISTANCE_RIGHT_TRACKING_WHEEL_CENTER;
    double radiusLeft = encoderLeft_data.delta_distance / deltaTheta + DISTANCE_LEFT_TRACKING_WHEEL_CENTER;
    
    // Calculate deltaX and deltaY during turning (robot moves in circular arcs)
    deltaDlocal.SetY(2.0 * std::sin(deltaTheta / 2.0) * radiusRight);
    deltaDlocal.SetX(2.0 * std::sin(deltaTheta / 2.0) * radiusLeft);
  }
  else {
    // If the robot is moving straight forward or backward, average encoder values for distance
    double deltaD = (encoderLeft_data.delta_distance + encoderRight_data.delta_distance) / 2.0;
    deltaDlocal.SetPosition(deltaD, 0);
  }
  
  // Updating angle
  SetRadians(GetRadians() + deltaTheta);
  
  // Updating global position
  SetPosition(GetX() + deltaDlocal.GetX() * std::cos(GetRadians()) - deltaDlocal.GetY() * std::sin(GetRadians()), 
              GetY() + deltaDlocal.GetX() * std::sin(GetRadians()) + deltaDlocal.GetY() * std::cos(GetRadians()));  

  // Save current values as previous for future updates
  encoderLeft_data.previous_value = encoderLeft_data.current_value;
  encoderRight_data.previous_value = encoderRight_data.current_value;
  encoderLeft_data.previous_distance = encoderLeft_data.current_distance;
  encoderRight_data.previous_distance = encoderRight_data.current_distance;

  gyro_data.prevDegrees = gyro_data.currentDegrees;
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

    // pros::lcd::print(5, "Enc R: %.2f degrees",
    //                  encoderFront.get_position() / 100.0);
    // pros::lcd::print(6, "Enc B: %.2f degrees",
    //                  encoderBack.get_position() / 100.0);

    // For update 3
    pros::lcd::print(5, "Enc R: %.2f degrees",
                     encoderLeft.get_position() / 100.0);
    pros::lcd::print(6, "Enc B: %.2f degrees",
                     encoderRight.get_position() / 100.0);

    odometry::Update();
    pros::delay(10);
  }
}

}  // namespace aon::odometry

#endif  // AON_SENSING_ODOMETRY_HPP_
