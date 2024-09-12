#ifndef AON_CONSTANTS_HPP_
#define AON_CONSTANTS_HPP_

//     ___           __ _                    _   _
//    / __|___ _ _  / _(_)__ _ _  _ _ _ __ _| |_(_)___ _ _
//   | (__/ _ \ ' \|  _| / _` | || | '_/ _` |  _| / _ \ ' \ 
//    \___\___/_||_|_| |_\__, |\_,_|_| \__,_|\__|_\___/_||_|
//                       |___/
#define BRAIN_SCREEN_WIDTH 480
#define BRAIN_SCREEN_HEIGHT 240

// NOT using 15 inch robot = Using 18 inch robot
#define USING_15_INCH_ROBOT true

//    _ ___
//   / | __|
//   | |__ \
//   |_|___/
//
#if USING_15_INCH_ROBOT

#define TRACKING_WHEEL_DIAMETER 2.0
#define DEGREES_PER_REVOLUTION 360.0
#define INITIAL_ODOMETRY_X 0.0
#define INITIAL_ODOMETRY_Y 0.0
#define INITIAL_ODOMETRY_THETA 0.0
#define DISTANCE_LEFT_TRACKING_WHEEL_CENTER 6.38
#define DISTANCE_RIGHT_TRACKING_WHEEL_CENTER 6.38
#define DISTANCE_BACK_TRACKING_WHEEL_CENTER 6.25
#define GYRO_ENABLED false
#define GYRO_CONFIDENCE 1.0
#define GYRO_FILTER_LENGTH 1

#define DRIVE_WHEEL_DIAMETER 2.75
#define HOLONOMIC_MOTION_X_DISTANCE 5.25
#define HOLONOMIC_MOTION_Y_DISTANCE 6.125

#define MAX_SPEED \
  ((2.0 * M_PI / 60.0) * (DRIVE_WHEEL_DIAMETER / 2.0) * 600 * M_SQRT1_2)

//    _ ___
//   / ( _ )
//   | / _ \
//   |_\___/
//
#else

#define TRACKING_WHEEL_DIAMETER 2.0
#define DEGREES_PER_REVOLUTION 360.0
#define INITIAL_ODOMETRY_X 0.0
#define INITIAL_ODOMETRY_Y 0.0
#define INITIAL_ODOMETRY_THETA 0.0
#define DISTANCE_LEFT_TRACKING_WHEEL_CENTER 7.83
#define DISTANCE_RIGHT_TRACKING_WHEEL_CENTER 7.83
#define DISTANCE_BACK_TRACKING_WHEEL_CENTER 6.5
#define GYRO_ENABLED false
#define GYRO_CONFIDENCE 1.0
#define GYRO_FILTER_LENGTH 1

#define DRIVE_WHEEL_DIAMETER 3.25
#define HOLONOMIC_MOTION_X_DISTANCE 7.0
#define HOLONOMIC_MOTION_Y_DISTANCE 5.35

#define MAX_SPEED ((2.0 * M_PI / 60.0) * (DRIVE_WHEEL_DIAMETER / 2.0) * 600 * M_SQRT1_2)

#endif

#endif  // AON_CONSTANTS_HPP_
