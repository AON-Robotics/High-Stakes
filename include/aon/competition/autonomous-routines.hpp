#pragma once

#include <cmath>
#include <algorithm>
#include "../constants.hpp"
#include "../globals.hpp"
#include "../sensing/odometry.hpp"
#include "../controls/pid/pid.hpp"
#include "../controls/holonomic-motion.hpp"

namespace aon {

double initial_pos_x;
double initial_pos_y;
double initial_heading;

/**
 * \warning Test this function first, it is old code that seems to get the angle needed to turn as well as the distance from current point to target.
*/
std::pair<double, double> getStepsTo(double desired_x, double desired_y)
{
  //3 seconds
  // odometry::ResetInitial();

  // Leave 500ms stationary to take the best results
  pros::delay(500);
  // initial_pos_x = gps.get_status().x;
  // initial_pos_y = gps.get_status().y;
  // initial_heading = gps.get_heading();

  double dist = sqrt(pow((desired_x - gps.get_status().x), 2) + pow((desired_y - gps.get_status().y), 2));

  double angle_destination = atan2(desired_y - gps.get_status().y, desired_x - gps.get_status().x) * 180.0 / M_PI;

  angle_destination = fmod(-angle_destination + 90, 360);
  
  double robot_heading = fmod(-gps.get_heading() + 90, 360);

  // Angle difference
  double rot = angle_destination - robot_heading;

  if (rot > 180) {
    rot -= 360;
  } 
  else if (rot < -180) {
    rot += 360;
  }

  return std::make_pair(dist, rot);
}

#if USING_15_INCH_ROBOT

int move(double dist);
int turn(double angle);
double metersToInches(double meters);

/**
 * \brief Resets odometry and gyro for error accumulation cleanse
 * 
 * \warning This function takes three (3) \b seconds to complete
 */
int initialReset(bool gyro = true)
{
  odometry::ResetInitial();
  
  // 3 seconds
  gyroscope.reset(gyro);
  return 1;
}

// ============================================================================
//    ___   _   _    ___ _   _ _      _ _____ ___ ___  _  _ ___  
//   / __| /_\ | |  / __| | | | |    /_\_   _|_ _/ _ \| \| / __| 
//  | (__ / _ \| |_| (__| |_| | |__ / _ \| |  | | (_) | .` \__ \ 
//   \___/_/ \_\____\___|\___/|____/_/ \_\_| |___\___/|_|\_|___/ 
//
// ============================================================================


/**
 * \brief Calculates time for the robot to reach a given distance
 * 
 * \param distance Distance from the robot to the target (remains constant) in \b inches
 * \returns The approximate time necessary to reach the target (overestimation) in \b seconds
 */
inline double getTimetoTarget(const double &distance){
  double circumference = DRIVE_WHEEL_DIAMETER * M_PI; // of the drive wheel (inches)
  double RPS = (int)driveRight.getGearing() / 60; // (revolutions per seconds)
  double velocity = RPS * circumference; // Calculated speed (in / s)
  double time = 2 * distance / velocity; // Calculated time (seconds)
  return time;
}

/**
 * \brief Calculates time for the robot to turn an angle
 * 
 * \param radians Angle remaining from the robot's current angle to the target (remains constant) in \b radians
 * 
 * \details The arc length formula is used as s = theta * radius (theta in radians)
 * 
 * \returns The approximate time necessary to reach the target (overestimation) in \b seconds
 * 
 */
inline double getTimetoTurnRad(const double &radians){
  double arcLength = radians * AVG_DRIVETRAIN_RADIUS; // Of the turn (inches)
  double circumference = DRIVE_WHEEL_DIAMETER * M_PI; // of the drive wheel (inches)
  double RPS = (int)driveRight.getGearing() / 60; // (revolutions per seconds)
  RPS /= 2; // We are using half power to turn
  double velocity = RPS * circumference; // Calculated speed (in / s)
  double time = 2 * arcLength / velocity; // Calculated time (seconds)
  return time;
}

/**
 * \brief Calculates time for the robot to turn an angle
 * 
 * \param degrees Angle remaining from the robot's current angle to the target (remains constant) in \b degrees
 * 
 * \returns The approximate time necessary to reach the target (overestimation) in \b seconds
 * 
 */
inline double getTimetoTurnDeg(const double &degrees) { return getTimetoTurnRad(degrees * M_PI / 180); }

/**
 * \brief Gets an average of the GPS position
 * 
 * \returns A Vector representing the position of the robot
 */
inline Vector getGPSPos(){
  double avg_x = 0;
  double avg_y = 0;
  
  // Get the average of the readings from the GPS
  for (int i = 0; i < GPS_SAMPLE_SIZE; i++)
  {
    avg_x += gps.get_status().x;
    avg_y += gps.get_status().y;
  }

  avg_x /= GPS_SAMPLE_SIZE; avg_y /= GPS_SAMPLE_SIZE;
  
  return Vector().SetPosition(avg_x, avg_y);
}

/**
 * \brief Determines the angle needed to turn 
 * 
 * \param target The point towards which we want to face
 * 
 * \returns The angle needed to turn in order to face the target
 * 
 * \warning The units must be meters because a mandatory conversion happens (this can be modified)
 * 
 * \todo Complete calculations
 */
inline double getAngleToTurn(Vector target){
  target.SetX(metersToInches(target.GetX()));
  target.SetY(metersToInches(target.GetY()));

  double heading = gps.get_heading();
  
  // Vector current = getGPSPos();
  Vector current = odometry::GetPosition(); // If this one is used the INITIAL_ODOMETRY_{COMPONENT} variables must be set and no reset can be done

  double result = 0;

  // Do corresponding calculations

  double toTarget = (target - current).GetDegrees(); // This number is in reference to the common cartesian plane if odometry position is used


  return result;
}

/**
 * \brief Conversion from \b meters to \b inches
 * 
 * \param meters The \b meters to be converted
 * 
 * \returns The distance in \b inches
 */
inline double metersToInches(double meters) { return meters * 39.3701; }

/**
 * \brief Determines the adjustment necessary to the voltage or speed being passed into drivetrain in order to pick up an object
 * 
 * \param sig The vision signature of the object to be recollected
 * 
 * \returns An integer representing the adjustment to be applied (by adding or subtracting) to voltage or speed of drivetrain 
 * 
 * \warning UNVERIFIED AND UNTESTED
 */
int trackObject(int sig = 1) {
    int adjustment = 10;
    pros::vision_object_s_t obj = vision_sensor.get_by_sig(0, sig);
    if (obj.signature == sig) {
        if (obj.x_middle_coord < -10) {
            // Turn left
            // return obj.x_middle_coord; // This could be a good adjustment for the robot's drivetrain, figure it out experimentally
            return -adjustment; // DUMMY VALUE FOR TESTS
        } else if (obj.x_middle_coord > 10) {
            // Turn right
            // return obj.x_middle_coord; // This could be a good adjustment for the robot's drivetrain, figure it out experimentally
            return adjustment; // DUMMY VALUE FOR TESTS
        } else {
            return 0;
        }
    }
    return 0;
}

// ============================================================================
//   __  __  _____   _____ __  __ ___ _  _ _____ 
//  |  \/  |/ _ \ \ / / __|  \/  | __| \| |_   _|
//  | |\/| | (_) \ V /| _|| |\/| | _|| .` | | |  
//  |_|  |_|\___/ \_/ |___|_|  |_|___|_|\_| |_|  
//
// ============================================================================


/**
 * \brief Moves the robot a given distance (default forward)
 * 
 * \param pid The PID used for the driving
 * \param dist The distance to be moved in \b inches
 */
void MoveDrivePID(PID pid = drivePID, double dist = TILE_WIDTH, const double MAX_SPEED = 100.0) {
  const int sign = dist / abs(dist); // Getting the direction of the movement
  dist = abs(dist); // Setting the magnitude to positive

  // Define the initialPos using the GPS instead of odometry (later should be both)
  // aon::Vector initialPos = getGPSPos();
  pid.Reset();
  aon::Vector initialPos = aon::odometry::GetPosition();

  const double timeLimit = getTimetoTarget(dist);
  const double start_time = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - start_time //every time the variable is called it is recalculated automatically

  while (time < timeLimit) {
    aon::odometry::Update();

    double currentDisplacement = (aon::odometry::GetPosition() - initialPos).GetMagnitude();

    double output = pid.Output(dist, currentDisplacement);

    pros::lcd::print(0, "%f", currentDisplacement);

    driveFull.moveVelocity(sign * std::clamp(output * (int)driveLeft.getGearing(), -MAX_SPEED, MAX_SPEED));

    pros::delay(10);
  }

  // Stop the motors
  driveLeft.moveVelocity(0);
  driveRight.moveVelocity(0);

  #undef time
}

/**
 * \brief Moves the robot toward a given position (default forward)
 * 
 * \param pid The PID used for the driving
 * \param targetPos A vector representing the position towards which we want to go
 * \param sign Determines the direction of the movement, 1 is front and -1 is backwards
 * 
 * \warning LEGACY
 */
void MoveDrivePID(PID pid, Vector targetPos, double sign = 1) {
  // Define the initialPos using the GPS instead of odometry (later should be both)
  // aon::Vector initialPos = getGPSPos();
  pid.Reset();
  aon::Vector initialPos = aon::odometry::GetPosition();

  const double timeLimit = getTimetoTarget(std::abs((targetPos - initialPos).GetMagnitude()));
  const double start_time = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - start_time //every time the variable is called it is recalculated automatically

  //Target position minus initial position
  const double targetDiplacement = (targetPos - initialPos).GetMagnitude();
  while (time < timeLimit) {
    aon::odometry::Update();

    double currentDisplacement = (aon::odometry::GetPosition() - initialPos).GetMagnitude();

    double output = pid.Output(targetDiplacement, currentDisplacement);

    pros::lcd::print(0, "%f", currentDisplacement);

    driveFull.moveVelocity(sign * std::clamp(output * (int)driveLeft.getGearing(), -100.0, 100.0));

    pros::delay(10);
  }

  // Stop the movement
  driveLeft.moveVelocity(0);
  driveRight.moveVelocity(0);

  #undef time
}


/**
 * \brief Turns the robot by a given angle (default clockwise)
 * 
 * \param pid The PID to be used for the turn
 * \param angle The angle to make the robot turn in \b degrees
 */
void MoveTurnPID(PID pid = turnPID, double angle = 90, const double MAX_SPEED = 50.0){
  const int sign = angle/abs(angle); // Getting the direction of the movement
  angle = abs(angle); // Setting the magnitude to positive
  pid.Reset();
  gyroscope.tare(); // .tare() or .reset(true) depending on the time issue
  const double startAngle = gyroscope.get_heading(); // Angle relative to the start

  if(sign == -1) { angle = 360.0 - angle + CLOCKWISE_ROTATION_DEGREES_OFFSET; }
  if(sign == 1) { angle -= CLOCKWISE_ROTATION_DEGREES_OFFSET; }

  const double targetAngle = angle;

  double timeLimit = getTimetoTurnDeg(targetAngle);

  const double startTime = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - startTime

  while(time < timeLimit){
    aon::odometry::Update();

    double traveledAngle = gyroscope.get_heading() - startAngle;

    double output = std::abs(pid.Output(targetAngle, traveledAngle)); //Use the absolute value of the output because if not, counter-clockwise turning is weird (error)

    pros::lcd::print(0, "%f", traveledAngle);

    // Taking clockwise rotation as positive (to change this just flip the negative on the sign below)
    driveLeft.moveVelocity(sign * std::clamp(output * (int)driveLeft.getGearing(), -MAX_SPEED, MAX_SPEED));
    driveRight.moveVelocity(-sign * std::clamp(output * (int)driveRight.getGearing(), -MAX_SPEED, MAX_SPEED));

    pros::delay(10);
  }

  driveLeft.moveVelocity(0);
  driveRight.moveVelocity(0);

  #undef time
}


// ============================================================================
//   ___ ___ __  __ ___ _    ___   __  __  _____   _____ __  __ ___ _  _ _____ 
//  / __|_ _|  \/  | _ \ |  | __| |  \/  |/ _ \ \ / / __|  \/  | __| \| |_   _|
//  \__ \| || |\/| |  _/ |__| _|  | |\/| | (_) \ V /| _|| |\/| | _|| .` | | |  
//  |___/___|_|  |_|_| |____|___| |_|  |_|\___/ \_/ |___|_|  |_|___|_|\_| |_|  
//
// ============================================================================


/**
 * \brief Moves the robot straight accross a given amount of tiles
 * 
 * \param amt The amount of tiles to be driven
 * 
 * \attention To move in reverse make the amount negative
 * 
 * \warning Robot should be aligned properly to achieve desired result
 */
void moveTilesStraight(int amt = 1) {
  move(TILE_WIDTH * amt);
}

/**
 * \brief Moves the robot straight accross a given amount of half-tiles
 * 
 * \param amt The amount of half-tiles to be driven
 * 
 * \attention To move in reverse make the amount negative
 * 
 * \warning Robot should be aligned properly to achieve desired result
 */
void moveHalfTiles(int amt = 1) {
  move((TILE_WIDTH / 2) * amt);
}

/**
 * \brief Moves the robot diagonally accross a given amount of tiles
 * 
 * \param amt The amount of tiles to be driven
 * 
 * \attention To move in reverse make the amount negative
 * 
 * \warning Robot should be aligned properly to achieve desired result
 */
void moveTilesDiag(int amt = 1) {
  move(TILE_DIAG_LENGTH * amt);
}

/**
 * \brief Moves the robot diagonally accross a given amount of half-tiles
 * 
 * \param amt The amount of half-tiles to be driven
 * 
 * \attention To move in reverse make the amount negative
 * 
 * \warning Robot should be aligned properly to achieve desired result
 */
void moveHalfDiagTiles(int amt = 1) {
  move((TILE_DIAG_LENGTH / 2) * amt);
}

/**
 * \brief Turns the robot clockwise by 90
 * 
 * \param amt The amount of 90 degree turns to make
 * 
 * \attention To move in reverse make the amount negative
 */
void turn90(int amt = 1){
  turn(90 * amt);
}

/**
 * \brief Turns to face a given target naively
 * 
 * \param target The point which the robot is supposed to face
 * 
 * \returns The angle between your current orientation and the target direction
 * 
 * \warning The units must be meters because a mandatory conversion happens (this can be modified)
 * \warning There are two turns meaning inefficiency
 */
void dumbTurnToTarget(Vector target){
  target.SetX(metersToInches(target.GetX()));
  target.SetY(metersToInches(target.GetY()));

  // Vector current = getGPSPos();
  Vector current = odometry::GetPosition(); // If this one is used the INITIAL_ODOMETRY_{COMPONENT} variables must be set and no reset can be done

  // These two lines are to put the robot in default position for the calculations after (common cartesian plane)
  double heading = gps.get_heading();
  turn(-1 * (heading + 90));

  double toTarget = (target - current).GetDegrees(); // This number is in respect to the common cartesian plane if odometry position is used

  turn(toTarget);
}

/**
 * \brief Moves the robot a given distance
 * 
 * \param dist The distance to move in \b inches
 */
int move(double dist = TILE_WIDTH)
{
  MoveDrivePID(drivePID, dist);
  drivePID.Reset();
  turnPID.Reset();
  // pros::delay(500);
  return 1;
}

/**
 * \brief Turn the robot a given angle (default is clockwise)
 * 
 * \param angle The angle to turn in \b degrees
 * 
 * \details Clockwise is positive and counter-clockwise is negative
 */
int turn(double angle = 90)
{
  // gyroscope.reset(true);
  // pros::delay(3000);
  MoveTurnPID(turnPID, angle);
  drivePID.Reset();
  turnPID.Reset();
  // pros::delay(500);
  return 1;
}


// ============================================================================
//   _____ ___ ___ _____ ___ 
//  |_   _| __/ __|_   _/ __|
//    | | | _|\__ \ | | \__ \
//    |_| |___|___/ |_| |___/
//
// ============================================================================

// Does not work yet because of getStepsTo()
void testGPS(int x, int y){ // Later name this function to GoTo
  std::pair<double, double> nextSteps = getStepsTo(x, y);
  turn(nextSteps.second);
  move(metersToInches(nextSteps.first / 1000));
}

inline void first_routine(double kP, double kI, double kD) {
  aon::PID pid = aon::PID(kP, kI, kD);
  int dist = 36; // Inches
  aon::Vector target = aon::Vector().SetPosition(dist, 0);
  
  aon::MoveDrivePID(pid, target);
  pid.Reset();

  intake.moveVelocity(90);
  pros::delay(500);
  intake.moveVelocity(0);

  target = aon::Vector().SetPosition(0, 0);
  aon::MoveDrivePID(pid, target, -1);

  pros::delay(500);
  // aon::turn90(pid);


  pid.Reset();
}

inline void turnTest(double kP, double kI, double kD) {
  PID pid = PID(kP, kI, kD);
  for(int i = 0; i < 4; i++){
    aon::MoveTurnPID(pid, 90);
    pros::delay(500);
  }
  // aon::MoveTurnPID(pid, 90, -1);
  // pros::delay(500);
  // aon::MoveTurnPID(pid, 45, -1);
  // pros::delay(500);
  pid.Reset();
}

inline void programming_skills() {
  first_routine(10, 0, 0);
  // drive_left.moveVelocity(-100);
  // drive_right.moveVelocity(-100);
  pros::delay(1500);
  // drive_right.moveVelocity(0);
  // aon::PID pid = aon::PID(10, 0, 0);
  // aon::Vector target = aon::Vector().SetPosition(-2.0, 0);
  // aon::MoveDrivePID(pid, target, 2);
  // pid.Reset();

  /*target = aon::Vector().SetPosition(0, 0);
  aon::MoveDrivePID(f_pid, target, 3, -1);
  f_id.Reset();*/

  // expansion.set_value(1);

}
  int first_routine_wrapper() {  // fixing gui return type Temp
  // aon::odometry::Debug();
  first_routine(10, 0, 0);
  return 0;
  
}

void tempRoutine() {  // temporary routine to test GUI

  // aon::PID x_pid = aon::PID(10, 0, 0.75);
  // aon::PID y_pid = aon::PID(10, 0, 0.75);
  // aon::PID heading_pid = aon::PID(0.15, 0, 0.0099);

  // MoveDrivePID(x_pid, y_pid, heading_pid, 10, 0, 0, 3);
  // x_pid.Reset();
  // y_pid.Reset();
  // heading_pid.Reset();
}

int tempRoutine_wrapper() {  // fixing gui return type Temp
  // aon::odometry::Debug();
  // tempRoutine();
  aon::programming_skills();
  // driveLeft.moveVoltage(12000);
  // driveRight.moveVoltage(12000);
  return 0;
}

// TESTING PHASE FOR PID

int proportionalFavoredRoutine(){
  first_routine(10, 0, 0);
  return 0;
}

int integralFavoredRoutine(){
  first_routine(0, 10, 0);
  return 0;
}

// This one causes no movement because it depends on change of error (there is none)
int derivativeFavoredRoutine(){
  first_routine(0, 0, 10);
  return 0;
}

void squareRoutine(){
  PID drivePID = PID(0.1, 0, 0);
  PID turnPID = PID(0.01, 0, 0);
  const int dist = 12;
  //Draws a square with the robot
  for(int i = 0; i < 4; i++){
    odometry::ResetInitial();
    // MoveDrivePID(drivePID, Vector().SetPosition(dist, 0));
    move(dist);
    // drivePID.Reset();
    // turnPID.Reset();

    // gyroscope.reset(true);
    MoveTurnPID(turnPID, 90);
    // drivePID.Reset();
    // turnPID.Reset();

    pros::delay(500);
  }
}

int combinationPIDRoutine(){
  // squareRoutine();
  // first_routine(0.1, 0, 0);
  turnTest(0.01, 0, 0);
  return 0;
}

// void Vision_Sensor(){
//   int cnt= vision_sensor.get_object_count();
//   pros::vision_object_s_t return_obj= vision_sensor.get_by_sig(0,1);
//   if( return_obj.width>5 && cnt>0){
//     rail.moveVelocity(100);

//   }
//   else{
//     rail.moveVelocity(200);
//   }
//   pros::delay(200);
// }

// END TEST FUNCTIONS

// ============================================================================|
//   ___  ___  _   _ _____ ___ _  _ ___ ___                                    |
//  | _ \/ _ \| | | |_   _|_ _| \| | __/ __|                                   |
//  |   / (_) | |_| | | |  | || .` | _|\__ \                                   |
//  |_|_\\___/ \___/  |_| |___|_|\_|___|___/                                   |
//                                                                             |
// ============================================================================|

/**
 * \brief This routine is if WE ARE BLUE and want to grab RED RINGS
*/
int enemyRingsRoutine(){
  // This routine focuses on enemy rings (no duh)
  // Rush to one of the side mobile goals (the one on the side of the negative points) and secure it on team side
  
  // Starting point at (1.5, .3) facing approximately 110 degrees (the blue area)
  raceToGoal(45);

  // Grab and secure
  moveHalfDiagTiles(1);
  piston.set_value(false);

  // Go to middle mobile goal to secure it
  turn(-100);
  moveTilesStraight(-1);
  // Grab and secure
  grabGoal();
  moveTilesDiag(1);

  // Stack enemy rings on mobile goal
  // Take mobile goal to negative points area
  // Go to first secured mobile goal
  // Stack team rings on mobile goal with remaining time
  return 1;
}

/**
 * \brief This routine is if WE ARE BLUE and want to grab BLUE RINGS
*/
int teamRingsRoutine(){
  // This routine focuses on team rings (no duh)
  // Rush to one of the side mobile goals (the one on the side of the double points) and secure it on team side
  // Stack team rings on mobile goal
  // Take mobile goal to double points area
  // Go to mobile goal that started in team area (not one of the three contested ones)
  // Stack team rings on mobile goal with remaining time
  return 1;
}

/**
 * \brief This small subroutine moves the intake such that a ring is scored on the mobile goal being carried
 * 
 * \param delay The time in \b milliseconds to leave the intake running
*/
void pickUpRing(int delay = 1000){
  intake.moveVelocity(INTAKE_VELOCITY);
  pros::delay(delay);
  intake.moveVelocity(0);
}

/**
 * \brief This subroutine follows an object (in our case a ring) with a given color signature and picks it up
 * 
 * \param SIGNATURE The id number of the vision signature of the object to follow and pick up
 * 
 * \todo Add time constraint in case a ring is never found
*/
void driveIntoRing(const int SIGNATURE){
  const int TOLERANCE = 20;
  const int VISION_FIELD_CENTER = 315 / 2;
  const int SPEED = 150; // 200 is max
  const int ADJUSTMENT = 20;
  while(true){
    auto object = vision_sensor.get_by_sig(0, SIGNATURE);
    const int OBJ_CENTER = object.x_middle_coord;

    if(object.signature == SIGNATURE){
      if(abs(OBJ_CENTER - VISION_FIELD_CENTER) <= TOLERANCE){
        driveFull.moveVelocity(SPEED);
      }
      else if(OBJ_CENTER < VISION_FIELD_CENTER){ // TURN LEFT
        driveLeft.moveVelocity(SPEED - ADJUSTMENT);
        driveRight.moveVelocity(SPEED + ADJUSTMENT);
      }
      else if(OBJ_CENTER > VISION_FIELD_CENTER){ // TURN RIGHT
        driveLeft.moveVelocity(SPEED + ADJUSTMENT);
        driveRight.moveVelocity(SPEED - ADJUSTMENT);
      }

      if(distanceSensor.get() <= 100){
        driveFull.moveVelocity(100);
        pickUpRing(1000);
        break;
      }
    }

    if(main_controller.get_digital(DIGITAL_B)){ // Safety During testing
      driveFull.moveVelocity(0);
      intake.moveVelocity(0);
      return;
    }
  }
  driveFull.moveVelocity(0);
  pickUpRing(1500); // Remember to do this after to finish pickup
}

/**
 * \brief This small subroutine grabs a goal (stake)
 * 
 * \warning You must already be very close to the goal and facing away (with the clamp towards it)
 * 
 * \details This routine uses timing but ideally there would be a way of knowing when we have the goal within our grasp
*/
void grabGoal(){
  driveFull.moveVelocity(-100);
  pros::delay(500);
  piston.set_value(true);
  pros::delay(100);
  driveFull.moveVelocity(100);
  pros::delay(600);
  driveFull.moveVelocity(0);
}

/**
 * \brief This subroutine moves toward a mobile goal IN REVERSE
 * 
 * \param dist This is the absolute value of the distance the mobile goal is from the robot in \b inches
 * 
 * \details The function already converts the distance to negative so the robot drives into the goal backwards
 * 
 * \todo Change the internal move() function to directly use the MoveDrivePid() function with a specific PID and speed
*/
void raceToGoal(double dist){
  dist = abs(dist);
  move(-dist);
  grabGoal();
}


#else

int move(double dist);
int turn(double angle);
double metersToInches(double meters);

/**
 * \brief Resets odometry and gyro for error accumulation cleanse
 * 
 * \warning This function takes three (3) \b seconds to complete
 */
int initialReset(bool gyro = true)
{
  odometry::ResetInitial();
  
  // 3 seconds
  gyroscope.reset(gyro);
  return 1;
}

// ============================================================================
//    ___   _   _    ___ _   _ _      _ _____ ___ ___  _  _ ___  
//   / __| /_\ | |  / __| | | | |    /_\_   _|_ _/ _ \| \| / __| 
//  | (__ / _ \| |_| (__| |_| | |__ / _ \| |  | | (_) | .` \__ \ 
//   \___/_/ \_\____\___|\___/|____/_/ \_\_| |___\___/|_|\_|___/ 
//
// ============================================================================


/**
 * \brief Calculates time for the robot to reach a given distance
 * 
 * \param distance Distance from the robot to the target (remains constant) in \b inches
 * \returns The approximate time necessary to reach the target (overestimation) in \b seconds
 */
inline double getTimetoTarget(const double &distance){
  double circumference = DRIVE_WHEEL_DIAMETER * M_PI; // of the drive wheel (inches)
  double RPS = (int)driveRight.getGearing() / 60; // (revolutions per seconds)
  double velocity = RPS * circumference; // Calculated speed (in / s)
  double time = 2 * distance / velocity; // Calculated time (seconds)
  return time;
}

/**
 * \brief Calculates time for the robot to turn an angle
 * 
 * \param radians Angle remaining from the robot's current angle to the target (remains constant) in \b radians
 * 
 * \details The arc length formula is used as s = theta * radius (theta in radians)
 * 
 * \returns The approximate time necessary to reach the target (overestimation) in \b seconds
 * 
 */
inline double getTimetoTurnRad(const double &radians){
  double arcLength = radians * AVG_DRIVETRAIN_RADIUS; // Of the turn (inches)
  double circumference = DRIVE_WHEEL_DIAMETER * M_PI; // of the drive wheel (inches)
  double RPS = (int)driveRight.getGearing() / 60; // (revolutions per seconds)
  RPS /= 2; // We are using half power to turn
  double velocity = RPS * circumference; // Calculated speed (in / s)
  double time = 2 * arcLength / velocity; // Calculated time (seconds)
  return time;
}

/**
 * \brief Calculates time for the robot to turn an angle
 * 
 * \param degrees Angle remaining from the robot's current angle to the target (remains constant) in \b degrees
 * 
 * \returns The approximate time necessary to reach the target (overestimation) in \b seconds
 * 
 */
inline double getTimetoTurnDeg(const double &degrees) { return getTimetoTurnRad(degrees * M_PI / 180); }

/**
 * \brief Gets an average of the GPS position
 * 
 * \returns A Vector representing the position of the robot
 */
inline Vector getGPSPos(){
  double avg_x = 0;
  double avg_y = 0;
  
  // Get the average of the readings from the GPS
  for (int i = 0; i < GPS_SAMPLE_SIZE; i++)
  {
    avg_x += gps.get_status().x;
    avg_y += gps.get_status().y;
  }

  avg_x /= GPS_SAMPLE_SIZE; avg_y /= GPS_SAMPLE_SIZE;
  
  return Vector().SetPosition(avg_x, avg_y);
}

/**
 * \brief Determines the angle needed to turn 
 * 
 * \param target The point towards which we want to face
 * 
 * \returns The angle needed to turn in order to face the target
 * 
 * \warning The units must be meters because a mandatory conversion happens (this can be modified)
 * 
 * \todo Complete calculations
 */
inline double getAngleToTurn(Vector target){
  target.SetX(metersToInches(target.GetX()));
  target.SetY(metersToInches(target.GetY()));

  double heading = gps.get_heading();
  
  // Vector current = getGPSPos();
  Vector current = odometry::GetPosition(); // If this one is used the INITIAL_ODOMETRY_{COMPONENT} variables must be set and no reset can be done

  double result = 0;

  // Do corresponding calculations

  double toTarget = (target - current).GetDegrees(); // This number is in reference to the common cartesian plane if odometry position is used


  return result;
}


/**
 * \brief Conversion from \b meters to \b inches
 * 
 * \param meters The \b meters to be converted
 * 
 * \returns The distance in \b inches
 */
inline double metersToInches(double meters) { return meters * 39.3701; }

/**
 * \brief Determines the adjustment necessary to the voltage or speed being passed into drivetrain in order to pick up an object
 * 
 * \param sig The vision signature of the object to be recollected
 * 
 * \returns An integer representing the adjustment to be applied (by adding or subtracting) to voltage or speed of drivetrain 
 * 
 * \warning UNVERIFIED AND UNTESTED
 */
int trackObject(int sig = 1) {
    int adjustment = 10;
    pros::vision_object_s_t obj = vision_sensor.get_by_sig(0, sig);
    if (obj.signature == sig) {
        if (obj.x_middle_coord < -10) {
            // Turn left
            // return obj.x_middle_coord; // This could be a good adjustment for the robot's drivetrain, figure it out experimentally
            return -adjustment; // DUMMY VALUE FOR TESTS
        } else if (obj.x_middle_coord > 10) {
            // Turn right
            // return obj.x_middle_coord; // This could be a good adjustment for the robot's drivetrain, figure it out experimentally
            return adjustment; // DUMMY VALUE FOR TESTS
        } else {
            return 0;
        }
    }
    return 0;
}

// ============================================================================
//   __  __  _____   _____ __  __ ___ _  _ _____ 
//  |  \/  |/ _ \ \ / / __|  \/  | __| \| |_   _|
//  | |\/| | (_) \ V /| _|| |\/| | _|| .` | | |  
//  |_|  |_|\___/ \_/ |___|_|  |_|___|_|\_| |_|  
//
// ============================================================================


/**
 * \brief Moves the robot a given distance (default forward)
 * 
 * \param pid The PID used for the driving
 * \param dist The distance to be moved in \b inches
 */
void MoveDrivePID(PID pid = drivePID, double dist = TILE_WIDTH, const double MAX_SPEED = 100.0) {
  const int sign = dist / abs(dist); // Getting the direction of the movement
  dist = abs(dist); // Setting the magnitude to positive

  // Define the initialPos using the GPS instead of odometry (later should be both)
  // aon::Vector initialPos = getGPSPos();
  pid.Reset();
  aon::Vector initialPos = aon::odometry::GetPosition();

  const double timeLimit = getTimetoTarget(dist);
  const double start_time = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - start_time //every time the variable is called it is recalculated automatically

  while (time < timeLimit) {
    aon::odometry::Update();

    double currentDisplacement = (aon::odometry::GetPosition() - initialPos).GetMagnitude();

    double output = pid.Output(dist, currentDisplacement);

    pros::lcd::print(0, "%f", currentDisplacement);

    driveFull.moveVelocity(sign * std::clamp(output * (int)driveLeft.getGearing(), -MAX_SPEED, MAX_SPEED));

    pros::delay(10);
  }

  // Stop the motors
  driveLeft.moveVelocity(0);
  driveRight.moveVelocity(0);

  #undef time
}

/**
 * \brief Moves the robot toward a given position (default forward)
 * 
 * \param pid The PID used for the driving
 * \param targetPos A vector representing the position towards which we want to go
 * \param sign Determines the direction of the movement, 1 is front and -1 is backwards
 * 
 * \warning LEGACY
 */
void MoveDrivePID(PID pid, Vector targetPos, double sign = 1) {
  // Define the initialPos using the GPS instead of odometry (later should be both)
  // aon::Vector initialPos = getGPSPos();
  pid.Reset();
  aon::Vector initialPos = aon::odometry::GetPosition();

  const double timeLimit = getTimetoTarget(std::abs((targetPos - initialPos).GetMagnitude()));
  const double start_time = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - start_time //every time the variable is called it is recalculated automatically

  //Target position minus initial position
  const double targetDiplacement = (targetPos - initialPos).GetMagnitude();
  while (time < timeLimit) {
    aon::odometry::Update();

    double currentDisplacement = (aon::odometry::GetPosition() - initialPos).GetMagnitude();

    double output = pid.Output(targetDiplacement, currentDisplacement);

    pros::lcd::print(0, "%f", currentDisplacement);

    driveFull.moveVelocity(sign * std::clamp(output * (int)driveLeft.getGearing(), -100.0, 100.0));

    pros::delay(10);
  }

  // Stop the movement
  driveLeft.moveVelocity(0);
  driveRight.moveVelocity(0);

  #undef time
}

/**
 * \brief Turns the robot by a given angle (default clockwise)
 * 
 * \param pid The PID to be used for the turn
 * \param angle The angle to make the robot turn in \b degrees
 */
void MoveTurnPID(PID pid = turnPID, double angle = 90, const double MAX_SPEED = 50.0){
  const int sign = angle/abs(angle); // Getting the direction of the movement
  angle = abs(angle); // Setting the magnitude to positive
  pid.Reset();
  gyroscope.tare(); // .tare() or .reset(true) depending on the time issue
  const double startAngle = gyroscope.get_heading(); // Angle relative to the start

  if(sign == -1) { angle = 360.0 - angle + CLOCKWISE_ROTATION_DEGREES_OFFSET; }
  if(sign == 1) { angle -= CLOCKWISE_ROTATION_DEGREES_OFFSET; }

  const double targetAngle = angle;

  double timeLimit = getTimetoTurnDeg(targetAngle);

  const double startTime = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - startTime

  while(time < timeLimit){
    aon::odometry::Update();

    double traveledAngle = gyroscope.get_heading() - startAngle;

    double output = std::abs(pid.Output(targetAngle, traveledAngle)); //Use the absolute value of the output because if not, counter-clockwise turning is weird (error)

    pros::lcd::print(0, "%f", traveledAngle);

    // Taking clockwise rotation as positive (to change this just flip the negative on the sign below)
    driveLeft.moveVelocity(sign * std::clamp(output * (int)driveLeft.getGearing(), -MAX_SPEED, MAX_SPEED));
    driveRight.moveVelocity(-sign * std::clamp(output * (int)driveRight.getGearing(), -MAX_SPEED, MAX_SPEED));

    pros::delay(10);
  }

  driveLeft.moveVelocity(0);
  driveRight.moveVelocity(0);

  #undef time
}


// ============================================================================
//   ___ ___ __  __ ___ _    ___   __  __  _____   _____ __  __ ___ _  _ _____ 
//  / __|_ _|  \/  | _ \ |  | __| |  \/  |/ _ \ \ / / __|  \/  | __| \| |_   _|
//  \__ \| || |\/| |  _/ |__| _|  | |\/| | (_) \ V /| _|| |\/| | _|| .` | | |  
//  |___/___|_|  |_|_| |____|___| |_|  |_|\___/ \_/ |___|_|  |_|___|_|\_| |_|  
//
// ============================================================================


/**
 * \brief Moves the robot straight accross a given amount of tiles
 * 
 * \param amt The amount of tiles to be driven
 * 
 * \attention To move in reverse make the amount negative
 * 
 * \warning Robot should be aligned properly to achieve desired result
 */
void moveTilesStraight(int amt = 1) {
  move(TILE_WIDTH * amt);
}

/**
 * \brief Moves the robot straight accross a given amount of half-tiles
 * 
 * \param amt The amount of half-tiles to be driven
 * 
 * \attention To move in reverse make the amount negative
 * 
 * \warning Robot should be aligned properly to achieve desired result
 */
void moveHalfTiles(int amt = 1) {
  move((TILE_WIDTH / 2) * amt);
}

/**
 * \brief Moves the robot diagonally accross a given amount of tiles
 * 
 * \param amt The amount of tiles to be driven
 * 
 * \attention To move in reverse make the amount negative
 * 
 * \warning Robot should be aligned properly to achieve desired result
 */
void moveTilesDiag(int amt = 1) {
  move(TILE_DIAG_LENGTH * amt);
}

/**
 * \brief Moves the robot diagonally accross a given amount of half-tiles
 * 
 * \param amt The amount of half-tiles to be driven
 * 
 * \attention To move in reverse make the amount negative
 * 
 * \warning Robot should be aligned properly to achieve desired result
 */
void moveHalfDiagTiles(int amt = 1) {
  move((TILE_DIAG_LENGTH / 2) * amt);
}

/**
 * \brief Turns the robot clockwise by 90
 * 
 * \param amt The amount of 90 degree turns to make
 * 
 * \attention To move in reverse make the amount negative
 */
void turn90(int amt = 1){
  turn(90 * amt);
}

/**
 * \brief Turns to face a given target naively
 * 
 * \param target The point which the robot is supposed to face
 * 
 * \returns The angle between your current orientation and the target direction
 * 
 * \warning The units must be meters because a mandatory conversion happens (this can be modified)
 * \warning There are two turns meaning inefficiency
 */
void dumbTurnToTarget(Vector target){
  target.SetX(metersToInches(target.GetX()));
  target.SetY(metersToInches(target.GetY()));

  // Vector current = getGPSPos();
  Vector current = odometry::GetPosition(); // If this one is used the INITIAL_ODOMETRY_{COMPONENT} variables must be set and no reset can be done

  // These two lines are to put the robot in default position for the calculations after (common cartesian plane)
  double heading = gps.get_heading();
  turn(-1 * (heading + 90));

  double toTarget = (target - current).GetDegrees(); // This number is in respect to the common cartesian plane if odometry position is used

  turn(toTarget);
}

/**
 * \brief Moves the robot a given distance
 * 
 * \param dist The distance to move in \b inches
 */
int move(double dist = TILE_WIDTH)
{
  MoveDrivePID(drivePID, dist);
  drivePID.Reset();
  turnPID.Reset();
  // pros::delay(500);
  return 1;
}

/**
 * \brief Turn the robot a given angle (default is clockwise)
 * 
 * \param angle The angle to turn in \b degrees
 * 
 * \details Clockwise is positive and counter-clockwise is negative
 */
int turn(double angle = 90)
{
  // gyroscope.reset(true);
  // pros::delay(3000);
  MoveTurnPID(turnPID, angle);
  drivePID.Reset();
  turnPID.Reset();
  // pros::delay(500);
  return 1;
}


// ============================================================================
//   _____ ___ ___ _____ ___ 
//  |_   _| __/ __|_   _/ __|
//    | | | _|\__ \ | | \__ \
//    |_| |___|___/ |_| |___/
//
// ============================================================================

inline void first_routine(double kP, double kI, double kD) {
  aon::PID pid = aon::PID(kP, kI, kD);
  int dist = 36; // Inches
  aon::Vector target = aon::Vector().SetPosition(dist, 0);
  
  aon::MoveDrivePID(pid, target);
  pid.Reset();

  intake.moveVelocity(90);
  pros::delay(500);
  intake.moveVelocity(0);

  target = aon::Vector().SetPosition(0, 0);
  aon::MoveDrivePID(pid, target, -1);

  pros::delay(500);
  // aon::turn90(pid);


  pid.Reset();
}

inline void turnTest(double kP, double kI, double kD) {
  PID pid = PID(kP, kI, kD);
  for(int i = 0; i < 4; i++){
    aon::MoveTurnPID(pid, 90);
    pros::delay(500);
  }
  // aon::MoveTurnPID(pid, 90, -1);
  // pros::delay(500);
  // aon::MoveTurnPID(pid, 45, -1);
  // pros::delay(500);
  pid.Reset();
}

inline void programming_skills() {
  first_routine(10, 0, 0);
  // drive_left.moveVelocity(-100);
  // drive_right.moveVelocity(-100);
  pros::delay(1500);
  // drive_right.moveVelocity(0);
  // aon::PID pid = aon::PID(10, 0, 0);
  // aon::Vector target = aon::Vector().SetPosition(-2.0, 0);
  // aon::MoveDrivePID(pid, target, 2);
  // pid.Reset();

  /*target = aon::Vector().SetPosition(0, 0);
  aon::MoveDrivePID(f_pid, target, 3, -1);
  f_id.Reset();*/

  // expansion.set_value(1);

}
  int first_routine_wrapper() {  // fixing gui return type Temp
  // aon::odometry::Debug();
  first_routine(10, 0, 0);
  return 0;
  
}

void tempRoutine() {  // temporary routine to test GUI

  // aon::PID x_pid = aon::PID(10, 0, 0.75);
  // aon::PID y_pid = aon::PID(10, 0, 0.75);
  // aon::PID heading_pid = aon::PID(0.15, 0, 0.0099);

  // MoveDrivePID(x_pid, y_pid, heading_pid, 10, 0, 0, 3);
  // x_pid.Reset();
  // y_pid.Reset();
  // heading_pid.Reset();
}

int tempRoutine_wrapper() {  // fixing gui return type Temp
  // aon::odometry::Debug();
  // tempRoutine();
  aon::programming_skills();
  // driveLeft.moveVoltage(12000);
  // driveRight.moveVoltage(12000);
  return 0;
}

// TESTING PHASE FOR PID

int proportionalFavoredRoutine(){
  first_routine(10, 0, 0);
  return 0;
}

int integralFavoredRoutine(){
  first_routine(0, 10, 0);
  return 0;
}

// This one causes no movement because it depends on change of error (there is none)
int derivativeFavoredRoutine(){
  first_routine(0, 0, 10);
  return 0;
}

void squareRoutine(){
  PID drivePID = PID(0.1, 0, 0);
  PID turnPID = PID(0.01, 0, 0);
  const int dist = 12;
  //Draws a square with the robot
  for(int i = 0; i < 4; i++){
    odometry::ResetInitial();
    // MoveDrivePID(drivePID, Vector().SetPosition(dist, 0));
    move(dist);
    // drivePID.Reset();
    // turnPID.Reset();

    // gyroscope.reset(true);
    MoveTurnPID(turnPID, 90);
    // drivePID.Reset();
    // turnPID.Reset();

    pros::delay(500);
  }
}

int combinationPIDRoutine(){
  // squareRoutine();
  // first_routine(0.1, 0, 0);
  turnTest(0.01, 0, 0);
  return 0;
}

// void Vision_Sensor(){
//   int cnt= vision_sensor.get_object_count();
//   pros::vision_object_s_t return_obj= vision_sensor.get_by_sig(0,1);
//   if( return_obj.width>5 && cnt>0){
//     rail.moveVelocity(100);

//   }
//   else{
//     rail.moveVelocity(200);
//   }
//   pros::delay(200);
// }

// END TEST FUNCTIONS

// ============================================================================|
//   ___  ___  _   _ _____ ___ _  _ ___ ___                                    |
//  | _ \/ _ \| | | |_   _|_ _| \| | __/ __|                                   |
//  |   / (_) | |_| | | |  | || .` | _|\__ \                                   |
//  |_|_\\___/ \___/  |_| |___|_|\_|___|___/                                   |
//                                                                             |
// ============================================================================|

                                         
int teamRingsRoutine(){
  // This routine focuses on team rings (no duh)
  // Rush to one of the side mobile goals (the one on the side of the double points) and secure it on team side
  
  // Starting point at (2.5, 1.5) facing 90 degrees (the blue area)
  moveHalfTiles(-4);
  turn(-45);
  moveHalfDiagTiles(-1);

  // Grab and secure
  // driveFull.moveVoltage(-8000);
  piston.set_value(toggle(piston_on));
  // driveFull.moveVoltage(0);
  moveHalfDiagTiles(1);
  piston.set_value(toggle(piston_on));

  // Go to middle mobile goal to secure it
  turn(135);
  moveTilesStraight(-1);
  turn(-45);
  moveHalfDiagTiles(-1);

  // Grab and secure
  // driveFull.moveVoltage(-8000);
  piston.set_value(toggle(piston_on));
  // driveFull.moveVoltage(0);
  moveHalfDiagTiles(1);

  // Stack team rings on mobile goal
  // Take mobile goal to double points area
  // Go to first secured mobile goal
  // Stack team rings on mobile goal with remaining time
  return 1;
}

int enemyRingsRoutine(){
  // This routine focuses on enemy rings (no duh)
  // Rush to one of the side mobile goals (the one on the side of the negative points) and secure it on team side
  // Stack enemy rings on mobile goal
  // Take mobile goal to negative points area
  // Go to mobile goal that started in team area (not one of the three contested ones)
  // Stack team rings on mobile goal with remaining time
  return 1;
}


#endif
};  // namespace aon
