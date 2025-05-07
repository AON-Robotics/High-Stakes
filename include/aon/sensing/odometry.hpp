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
 *       - OFFSET_Y_ENCODER_MID (measure this)
 *       - DISTANCE_BACK_TRACKING_WHEEL_CENTER (measure this)
 *       - GYRO_CONFIDENCE (testing)    } should
 *       - ENCODER_CONFIDENCE (testing) } sum 1
 *       - DEGREES_PER_REVOLUTION    (= 360)
 *       - INITIAL_ODOMETRY_X ( = 0)
 *       - INITIAL_ODOMETRY_Y ( = 0)
 *       - INITIAL_ODOMETRY_THETA ( = 0)
 *       - GYRO_ENABLED ( = true | = false)
 *    2. Have available the `vector.hpp` header file
 *    3. Have available pros and okapilib
 *    4. Have available the `globals.hpp` header file with `encoderMid`,
 *    and `encoderBack` objects instantiated and `gyroscope` if
 *    the GYRO_ENABLED is true.
 *
 *  \par Instructions
 *    1. Call the `Initialize` function
 *    2. Call the `Update` function as frequently as possible to calculate the
 *    pose.
 *    3. Coordinate gyro with encoder back. Make sure when turning left, they are
 *       positive.
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
    double currentValue;     //> Current value in degrees
    double prevValue;    //> Previous value in degrees
    double delta;             //> previous_value - current_value

    //> All this 3 variables are in inches
    double currentDistance;  //> Distance that tracking wheel has traveled
    double previousDistance; //> Previous distance that tracking wheel has traveled
    double deltaDistance;    //> previous_distance - current_distance
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
  //> Conversion factor
  const double conversionFactor = M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION;
    
  //> Mutex for x position to prevent race condition when retrieving value
  pros::Mutex x_mutex;
  //> Mutex for y position to prevent race condition when retrieving value
  pros::Mutex y_mutex;
  //> Mutex for absolute position
  pros::Mutex p_mutex;
  //> Mutex for orientation to prevent race condition when retrieving value
  pros::Mutex orientation_mutex;
    
  // //> Encoder left struct instance
  // STRUCT_encoder encoderMid_data;
  // //> Encoder back struct instance
  // STRUCT_encoder encoderBack_data;
  //> Encoder left struct instance
  STRUCT_encoder encoderRight_data;
  //> Encoder back struct instance
  STRUCT_encoder encoderLeft_data;
  //> Gyro struct instance
  STRUCT_gyro gyro_data;

  // //> Get at what time we are starting 
  // static int startTime = pros::micros() / 1E6;
  // //> Current time during interaction 
  // static int time = 0;
  // //> Updates done by seconds
  // const int timeLimit = 1; 
  //> Accumulate values during this time
  double accumulatedTheta = 0;
  double accumulatedMid = 0;

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
  // const double currentAngleMid = encoderMid.get_position() / 100.0;
  // // const double currentAngleBack = encoderBack.get_position() / 100.0;
  // const double currentAngleGyro = gyroscope.get_heading();

  const double currentAngleRight = encoderRight.get_position() / 100.0;
  const double currentAngleLeft = encoderLeft.get_position() / 100.0;
  const double currentAngleGyro = gyroscope.get_heading();
  std::cout << "currentAngleGyro: " << currentAngleGyro << "\n";
  
  // // Reset encoder's struct variables 
  // encoderMid_data = {currentAngleMid,                       // current position in degrees
  //                      currentAngleMid,                      // previuos position in degrees
  //                      0,                                     // delta in degrees
  //                      currentAngleMid * conversionFactor,  // current position in inches 
  //                      currentAngleMid * conversionFactor,  // previuos position in inches
  //                      0.0};                                  // delta in inches

  // encoderBack_data = {currentAngleBack,                      // current position in degrees
  //                      currentAngleBack,                      // previuos position in degrees
  //                      0,                                      // delta in degrees
  //                      currentAngleBack * conversionFactor,  // current position in inches 
  //                      currentAngleBack * conversionFactor,  // previuos position in inches
  //                      0.0};                                   // delta in inches

  // Reset encoder's struct variables
  encoderRight_data = {currentAngleRight,                       // current position in degrees
                       currentAngleRight,                      // previuos position in degrees
                       0,                                     // delta in degrees
                       currentAngleRight * conversionFactor,  // current position in inches 
                       currentAngleRight * conversionFactor,  // previuos position in inches
                       0.0};                                  // delta in inches

  encoderLeft_data = {currentAngleLeft,                      // current position in degrees
                       currentAngleLeft,                      // previuos position in degrees
                       0,                                      // delta in degrees
                       currentAngleLeft * conversionFactor,  // current position in inches 
                       currentAngleLeft * conversionFactor,  // previuos position in inches
                       0.0};                                   // delta in inches

  gyro_data = {0,                               // current value degrees
               0,                               // prevuios value degrees
               0,              // current radians
               0.0,                                            // delta degrees
               0.0};                                           // delta radians
    
  // Preset odometry values
  deltaTheta = 0.0;
  deltaDlocal.SetPosition(0.0, 0.0);
  accumulatedTheta = 0;
  
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
  // encoderMid.set_position(0);
  // encoderBack.set_position(0);
    
  // encoderMid.reset();
  // encoderBack.reset();

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
 * \details Uses changes in encoder (middle and back) and gyro to calculate position
 * 
 * */

// for debbuging
double backTracker = 0;
int time = 0;
double minAngle[200];
double maxAngle = 0.000001;

bool turning = false;
bool arcMovementxd = false;

// inline void Update() {
//   // Read encoder values, divided by 100 to convert centidegrees to degrees
//   encoderMid_data.currentValue = encoderMid.get_position() / 100.0; 
//   // encoderBack_data.currentValue = -encoderBack.get_position() / 100.0; 

//   // Convert to distances
//   encoderMid_data.currentDistance = encoderMid_data.currentValue * conversionFactor;
//   // encoderBack_data.currentDistance = encoderBack_data.currentValue * conversionFactor;
  
//   // Calculate deltas
//   encoderMid_data.delta = encoderMid_data.currentValue - encoderMid_data.prevValue;
//   // encoderBack_data.delta = encoderBack_data.currentValue - encoderBack_data.prevValue;
  
//   encoderMid_data.deltaDistance = encoderMid_data.currentDistance - encoderMid_data.previousDistance;
//   // encoderBack_data.deltaDistance = encoderBack_data.currentDistance - encoderBack_data.previousDistance;

//   // Calculate delta theta if we dont have gyro
//   // double deltaThetaE = encoderBack_data.deltaDistance / DISTANCE_BACK_TRACKING_WHEEL_CENTER;
//   // backTracker += deltaThetaE;
  
//   // If we have gyro, get value and calculate delta 
//   #if GYRO_ENABLED 
//   // Read gyro value
//   gyro_data.currentDegrees = gyroscope.get_heading();
//   gyro_data.currentRadians = gyro_data.currentDegrees * (M_PI / 180);
  
//   // Calculate delta
//   gyro_data.deltaDegrees = gyro_data.currentDegrees - gyro_data.prevDegrees;
//   // std::cout << "gyro delta degrees: " << gyro_data.currentDegrees << " - " << gyro_data.prevDegrees << " = " << gyro_data.deltaDegrees << "\n";
  
//   // Save current data for future calculations
//   gyro_data.prevDegrees = gyro_data.currentDegrees;
  
//   // Normalize angle, put it between -180 and 180 (just for first interations)
//   if (gyro_data.deltaDegrees > 180) gyro_data.deltaDegrees -= 360; 
//   else if (gyro_data.deltaDegrees < -180) gyro_data.deltaDegrees += 360; 
  
//   gyro_data.deltaRadians = gyro_data.deltaDegrees * (M_PI / 180.0);
//   // std::cout << "Gyro delta radians: " << gyro_data.deltaRadians << "\n"; 
  
//   // Right now, confidence gyro 0.9, encoder cnfidence 0.1 (must sum 1) 
//   // deltaTheta = ENCODER_CONFIDENCE * deltaThetaE + GYRO_CONFIDENCE * gyro_data.deltaRadians;
//   deltaTheta = gyro_data.deltaRadians;
//   #endif
  
  
//   // Test, maybe only for test robot, make sure to update this for actual robots
//   // Take away fake reading in base on offset
//   // Calculations
//   time++;
//   std::cout << "\nTime: " << time << "\n";
  
//   std::cout << "Current distance | Prev value | Delta\n";
//   std::cout << encoderMid_data.currentDistance << " | " << encoderMid_data.previousDistance << " | " << encoderMid_data.deltaDistance << "\n";   
  
//   std::cout << "Current angle | Prev angle | Delta\n";
//   std::cout << gyro_data.currentDegrees << " | " << gyro_data.prevDegrees << " | " << gyro_data.deltaRadians * (180/M_PI) << "\n";
//   // std::cout << backTracker << " |        | " << deltaThetaE * (180/M_PI) << "\n";
//   // std::cout << "Delta theta: " << deltaTheta << "\n"; 
//   std::cout << "\n";
//   std::cout << "Current distance | Prev value | Delta\n";
//   std::cout << encoderMid_data.currentDistance << " | " << encoderMid_data.previousDistance << " | " << encoderMid_data.deltaDistance << "\n";   
  
//   std::cout << "Current angle | Prev angle | Delta\n";
//   std::cout << gyro_data.currentDegrees << " | " << gyro_data.prevDegrees << " | " << gyro_data.deltaRadians * (180/M_PI) << "\n";
//   // std::cout << backTracker << " |        | " << deltaThetaE * (180/M_PI) << "\n";
//   // std::cout << "Delta theta: " << deltaTheta << "\n"; 
//   std::cout << "\n";
  
//   if (encoderMid_data.deltaDistance > maxAngle) maxAngle = deltaTheta;
//   minAngle[time] = encoderMid_data.deltaDistance;
  
//   // std::cout << "Max Angle:   " << maxAngle << "\n"; 
//   // std::cout << "Min angle:   " << minAngle << "\n"; 
  
//   /**
//    *  Try implementing time
//    *  Or implement it with delta theta
//    * 
//    */
  
//   // Just update after accumulate 1.0 degrees
//   accumulatedTheta += deltaTheta;
//   accumulatedMid += encoderMid_data.deltaDistance;
//   // Update degrees fast
//   pros::lcd::print(1, "Accumulated theta: " , accumulatedTheta);
//   pros::lcd::print(2, "Accumulated mid: " , accumulatedMid);
  
  
  
//   // If we are rotating
//   if (std::abs(deltaTheta) > 0.01) {
//     // do the fix only when turning
//     std::cout << "WE'RE TURNING!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
//     // std::cout << "Fake     reading: " << encoderMid_data.deltaDistance << "\n"; 
//     // std::cout << "restando reading: " << OFFSET_X_ENCODER_MID * deltaTheta << "\n"; 
    
//     std::cout << "Delta theta in degrees: " << deltaTheta << "\n";
//     std::cout << "Acummulate theta in degrees: " << accumulatedTheta * (180/M_PI) << "\n";
//     // Just do an update every degree to maintain the math correct
//     if (std::abs(accumulatedTheta) >= THETA_THRESHOLD) {
//       std::cout << "THE ACCUMULATE THETA IS GREATER THAN THETA THERSHOLD, WE UPDATING\n";
//       double thetaToApply = accumulatedTheta;
//       accumulatedTheta = 0.0;
      
//       double midToApply = accumulatedMid;
//       accumulatedMid = 0.0;
      
//       // Calculate thereshold to determine if the movement is an arc one
//       double threshold = (OFFSET_X_ENCODER_MID + ARC_MOVEMENT_ALPHA) * std::abs(thetaToApply);
//       std::cout << "Encoder mid: "<< midToApply << ", and Threshold: " << threshold << "\n";
//       // pros::lcd::print(3, "Thershold: ", threshold);
      
      
//       if (std::abs(midToApply) > threshold) {
//         // std::cout << "There is arc movement\n";
//         // Move mid is how much to the front we go
//         double moveMid = (midToApply - (OFFSET_X_ENCODER_MID * thetaToApply)) / thetaToApply;
//         // std::cout << "Move mid: " << encoderMid_data.deltaDistance << "\n"; 
//         std::cout << "move mid: " << moveMid << ", theta: " << thetaToApply << "\n";
        
//         // Calculate deltaX and deltaY during turning (robot moves in circular arcs)
//         deltaDlocal.SetPosition(moveMid * (1 - cos(thetaToApply)), moveMid * sin(thetaToApply));
//       }
//       else {
//         std::cout << "There is NOOOOOOOOOOO arc movement\n";
//         deltaDlocal.SetPosition(0, 0);
//       }
      
//       std::cout << "Encoder delta: " << encoderMid_data.deltaDistance << "\n";
//       std::cout << "Threshold:     " << threshold << "\n\n";
      
//     } else {
//       std::cout << "ACCUMULATE THETA IS TOO SMALL, " << accumulatedTheta << " < " << THETA_THRESHOLD << "\n";
//     }
//   }
//   else {
//     // If the robot is moving straight forward or backward, average encoder values for distance
//     std::cout << "Not Rotating: delta theta: " << deltaTheta << "\n";
//     deltaDlocal.SetPosition(encoderMid_data.deltaDistance, 0);
//   }
  
//   // Updating angle
//   SetRadians(GetRadians() + deltaTheta);
//   // SetRadians(GetRadians() + deltaTheta);
  
//   // Updating global position using 2D transformation matrix
//   SetPosition(GetX() + deltaDlocal.GetX() * std::cos(GetRadians()) - deltaDlocal.GetY() * std::sin(GetRadians()), 
//   GetY() + deltaDlocal.GetX() * std::sin(GetRadians()) + deltaDlocal.GetY() * std::cos(GetRadians()));  
  
//   std::cout << "X: " << GetX() << ", Y: " << GetY() << ", Angle: " << GetDegrees() << "\n";
  
//   // Save current values as previous for future updates
//   encoderMid_data.prevValue = encoderMid_data.currentValue;
//   // encoderBack_data.prevValue = encoderBack_data.currentValue;
  
//   encoderMid_data.previousDistance = encoderMid_data.currentDistance;
//   // encoderBack_data.previousDistance = encoderBack_data.currentDistance;
  
//   // pros::delay(5000);
// }

void Update() {
  pros::lcd::print(1, "Updating odometry");
  
  /// Read encoder values, divided by 100 to convert centidegrees to degrees
  encoderRight_data.currentValue = -encoderRight.get_position() / 100.0; 
  encoderLeft_data.currentValue = -encoderLeft.get_position() / 100.0; 

  // Convert to distances
  encoderRight_data.currentDistance = encoderRight_data.currentValue * conversionFactor;
  encoderLeft_data.currentDistance = encoderLeft_data.currentValue * conversionFactor;

  // Calculate deltas
  encoderRight_data.delta = encoderRight_data.currentValue - encoderRight_data.prevValue;
  encoderLeft_data.delta = encoderLeft_data.currentValue - encoderLeft_data.prevValue;

  encoderRight_data.deltaDistance = encoderRight_data.currentDistance - encoderRight_data.previousDistance;
  encoderLeft_data.deltaDistance = encoderLeft_data.currentDistance - encoderLeft_data.previousDistance;

  // Calculate delta theta if we dont have gyro
  double deltaThetaA = (encoderLeft_data.deltaDistance - encoderRight_data.deltaDistance) / (DISTANCE_RIGHT_TRACKING_WHEEL_CENTER + DISTANCE_LEFT_TRACKING_WHEEL_CENTER);
  
  // if you want that when turn left is positive
  // double deltaThetaA = (encoderRight_data.deltaDistance - encoderLeft_data.deltaDistance) / (DISTANCE_RIGHT_TRACKING_WHEEL_CENTER + DISTANCE_LEFT_TRACKING_WHEEL_CENTER);

  backTracker += deltaThetaA;

  // If we have gyro, get value and calculate delta
  #if GYRO_ENABLED 
  // Read gyro value
  gyro_data.currentDegrees = gyroscope.get_heading();
  gyro_data.currentRadians = gyro_data.currentDegrees * (M_PI / 180);

  // Calculate delta
  gyro_data.deltaDegrees = gyro_data.currentDegrees - gyro_data.prevDegrees;
  if (gyro_data.deltaDegrees > 180) gyro_data.deltaDegrees -= 360; 
  else if (gyro_data.deltaDegrees < -180) gyro_data.deltaDegrees += 360; 
  
  gyro_data.deltaRadians = gyro_data.deltaDegrees * (M_PI / 180.0);

  std::cout << "Current distance | Prev value | Delta\n";
  std::cout << encoderLeft_data.currentDistance << " | " << encoderLeft_data.previousDistance << " | " << encoderLeft_data.deltaDistance << "\n";   
  std::cout << encoderRight_data.currentDistance << " | " << encoderRight_data.previousDistance << " | " << encoderRight_data.deltaDistance << "\n";   
  
  std::cout << "Current angle | Prev angle | Delta\n";
  std::cout << gyro_data.currentDegrees << " | " << gyro_data.prevDegrees << " | " << gyro_data.deltaRadians * (180/M_PI) << "\n";
  std::cout << "Delta by encoders: " << deltaThetaA << ", Total: " << backTracker << "\n";

  // Save current data for future calculations
  gyro_data.prevDegrees = gyro_data.currentDegrees;

  // Right now, confidence gyro 0.9, encoder confidence 0.1 (must sum 1) 
  deltaTheta = (1 - GYRO_CONFIDENCE) * deltaThetaA + GYRO_CONFIDENCE * gyro_data.deltaRadians;
  std::cout << "Delta Theta: " << deltaTheta * (180/M_PI) << "\n";
  std::cout << "\n";
  // deltaTheta = gyro_data.deltaRadians;
  #endif

  // If we are rotating
  if (std::abs(deltaTheta) > 0.01) {
    // Calculate the radius of rotation for each wheel
    if ((encoderLeft_data.deltaDistance * encoderRight_data.deltaDistance) <= 0) {
      // Turning in its own axis
      std::cout << "Turning in own axis, No movement. Update angle.\n";
      deltaDlocal.SetPosition(0.0, 0.0);
    }
    else {
      std::cout << "Arc movement\n";
      double radiusLeft;
      double radiusRight;
  
      // CHANGE DELTA DEPENDING OF WHAT IS POSITIVE
      double sign = (deltaTheta > 0) ? 1 : -1;
      radiusLeft  = (encoderLeft_data.deltaDistance / deltaTheta)  - sign * DISTANCE_LEFT_TRACKING_WHEEL_CENTER;
      radiusRight = (encoderRight_data.deltaDistance / deltaTheta) + sign * DISTANCE_RIGHT_TRACKING_WHEEL_CENTER;
      std::cout << "Radius: " << radiusLeft << ", " << radiusRight << "\n\n";

      double averageR = (encoderLeft_data.deltaDistance + encoderRight_data.deltaDistance) / (2.0 * deltaTheta);
  
      // Calculate deltaX and deltaY during turning (robot moves in circular arcs)
      // deltaDlocal.SetY(2.0 * std::sin(deltaTheta / 2.0) * radiusRight);
      // deltaDlocal.SetX(2.0 * std::sin(deltaTheta / 2.0) * radiusLeft);
      deltaDlocal.SetPosition(averageR * std::sin(deltaTheta), averageR * (1 - std::cos(deltaTheta)));
    }
  }
  else {
    // If the robot is moving straight forward or backward, average encoder values for distance
    std::cout << "There is no turning\n";
    double deltaD = (encoderLeft_data.deltaDistance + encoderRight_data.deltaDistance) / 2.0;
    deltaDlocal.SetPosition(deltaD, 0);
  } 

  // Updating angle
  SetRadians(GetRadians() + deltaTheta);

  // Updating global position
  SetPosition(GetX() + deltaDlocal.GetX() * std::cos(GetRadians()) - deltaDlocal.GetY() * std::sin(GetRadians()), 
              GetY() + deltaDlocal.GetX() * std::sin(GetRadians()) + deltaDlocal.GetY() * std::cos(GetRadians()));  

 std::cout << "X: " << GetX() << ", Y: " << GetY() << ", Angle: " << GetDegrees() << "\n\n";
 pros::lcd::print(1, "X: ", GetX());
 pros::lcd::print(1, "Y: ", GetY());
 pros::lcd::print(1, "Angle: ", GetDegrees());

  // Save current values as previous for future updates
  encoderLeft_data.prevValue = encoderLeft_data.currentValue;
  encoderRight_data.prevValue = encoderRight_data.currentValue;

  encoderRight_data.previousDistance = encoderRight_data.currentDistance;
  encoderLeft_data.previousDistance = encoderLeft_data.currentDistance;

  gyro_data.prevDegrees = gyro_data.currentDegrees;
}

void UpdateTurn(double angle) {
  std::cout << "Angle to Update: " << angle << "\n";
  SetDegrees(GetDegrees() + angle);
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
    pros::lcd::print(1, "X: %0.3f", GetX());
    pros::lcd::print(2, "Y: %0.3f", GetY());
    pros::lcd::print(3, "Heading: %0.3f", GetDegrees());
    // pros::lcd::print(2, "Gyro: %.2f degrees", gyro_data.currentDegrees);
    // pros::lcd::print(3, "Back Degrees: %.2f degrees", backTracker * (180/M_PI));
    // pros::lcd::print(5, "Accumulate Theta: ", accumulatedTheta);
                     

    odometry::Update();
    pros::delay(20);
  }
}

}  // namespace aon::odometry

#endif  // AON_SENSING_ODOMETRY_HPP_
