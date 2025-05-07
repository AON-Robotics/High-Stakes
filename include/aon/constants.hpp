#ifndef AON_CONSTANTS_HPP_
#define AON_CONSTANTS_HPP_

#define BRAIN_SCREEN_WIDTH 480
#define BRAIN_SCREEN_HEIGHT 240
#define DEGREES_PER_REVOLUTION 360.0
#define TILE_WIDTH 23.6220472441
#define TILE_DIAG_LENGTH 33.4066195836 // Calculated with the Pythagorean theorem
#define GPS_SAMPLE_SIZE 50

// Colors
#define RED 1
#define BLUE 2

// NOT using black robot = Using green robot
#define USING_BLACK_ROBOT true
#define TESTING_AUTONOMOUS false 
#define COLOR RED

#if USING_BLACK_ROBOT

#define SENSITIVITY 3 // 3-10 works good, currently undergoing testing // Higher is more sensitivity
#define DRIVE_WHEEL_DIAMETER 3.25
#define TRACKING_WHEEL_DIAMETER 1.959
#define DISTANCE_LEFT_TRACKING_WHEEL_CENTER 1.572
#define DISTANCE_RIGHT_TRACKING_WHEEL_CENTER 1.572
#define DISTANCE_BACK_TRACKING_WHEEL_CENTER 1.572
#define MOTOR_TO_DRIVE_RATIO .8 // NumTeethMotorGear / NumTeethWheelGear
#define GYRO_ENABLED true
#define GYRO_CONFIDENCE 1
#define GYRO_FILTER_LENGTH 1
#define ENCODER_CONFIDENCE 0
#define OFFSET_X_ENCODER_MID 3.250

#define DRIVE_WIDTH 12.15 //distance between front wheels
#define DRIVE_LENGTH 9 //distance from back wheel to front wheel
#define DISTANCE_FRONT_LEFT_DRIVE_WHEEL_CENTER 7.55
#define DISTANCE_BACK_LEFT_DRIVE_WHEEL_CENTER 7.55
#define DISTANCE_FRONT_RIGHT_DRIVE_WHEEL_CENTER 7.55 //PYTHAG
#define DISTANCE_BACK_RIGHT_DRIVE_WHEEL_CENTER 7.55
#define AVG_DRIVETRAIN_RADIUS (DISTANCE_FRONT_LEFT_DRIVE_WHEEL_CENTER + DISTANCE_BACK_LEFT_DRIVE_WHEEL_CENTER + DISTANCE_FRONT_RIGHT_DRIVE_WHEEL_CENTER + DISTANCE_BACK_RIGHT_DRIVE_WHEEL_CENTER) / 4
// This number may be dependent on the degrees being turn in which case it will not be a constant
#define CLOCKWISE_ROTATION_DEGREES_OFFSET 0

// Depend on the robot and the routine
#define INITIAL_ODOMETRY_X 0.0
#define INITIAL_ODOMETRY_Y 0.0
#define INITIAL_ODOMETRY_THETA 0.0

// These next four (4) are in meters (all else is inches)
#define GPS_X_OFFSET 0 // CAD
#define GPS_Y_OFFSET 0.154304916675 // CAD
#define GPS_INITIAL_X -0.34 // Field
#define GPS_INITIAL_Y -0.82 // Field
#define GPS_INITIAL_HEADING 296.86 // Field (in Degrees)

#define MAX_RPM (int)driveFull.getGearing()
#define INTAKE_VELOCITY (int)intake.getGearing()


#else


#define SENSITIVITY 30 // 20 works good, currently undergoing testing
#define DRIVE_WHEEL_DIAMETER 3.25
#define TRACKING_WHEEL_DIAMETER 1.959
#define DISTANCE_LEFT_TRACKING_WHEEL_CENTER 5.650
#define DISTANCE_RIGHT_TRACKING_WHEEL_CENTER 5.650
#define DISTANCE_BACK_TRACKING_WHEEL_CENTER 5.650
#define GYRO_ENABLED true
#define GYRO_CONFIDENCE 1.0
#define GYRO_FILTER_LENGTH 1

#define DRIVE_WIDTH 12.478
#define DRIVE_LENGTH 10
#define DISTANCE_FRONT_LEFT_DRIVE_WHEEL_CENTER 7.99531869284
#define DISTANCE_BACK_LEFT_DRIVE_WHEEL_CENTER  7.99531869284
#define DISTANCE_FRONT_RIGHT_DRIVE_WHEEL_CENTER 7.99531869284
#define DISTANCE_BACK_RIGHT_DRIVE_WHEEL_CENTER 7.99531869284
#define AVG_DRIVETRAIN_RADIUS (DISTANCE_FRONT_LEFT_DRIVE_WHEEL_CENTER + DISTANCE_BACK_LEFT_DRIVE_WHEEL_CENTER + DISTANCE_FRONT_RIGHT_DRIVE_WHEEL_CENTER + DISTANCE_BACK_RIGHT_DRIVE_WHEEL_CENTER) / 4
// This number may be dependent on the degrees being turn in which case it will not be a constant
#define CLOCKWISE_ROTATION_DEGREES_OFFSET 0

// Depend on the robot and the routine
#define INITIAL_ODOMETRY_X 0.0
#define INITIAL_ODOMETRY_Y 0.0
#define INITIAL_ODOMETRY_THETA 0.0

// These next four (4) are in meters (all else is inches)
#define GPS_X_OFFSET -0.149225298451
#define GPS_Y_OFFSET 0.127000254001
#define GPS_INITIAL_X -0.34
#define GPS_INITIAL_Y 0.82
#define GPS_INITIAL_HEADING 245.7 // Degrees

#define INTAKE_VELOCITY (int)intake.getGearing()
#define RAIL_VELOCITY (int)rail.getGearing()


#endif

#endif  // AON_CONSTANTS_HPP_
