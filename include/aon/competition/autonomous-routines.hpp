#pragma once

#include <cmath>
#include <algorithm>
#include "../constants.hpp"
#include "../globals.hpp"
#include "../sensing/odometry.hpp"
#include "../controls/pid/pid.hpp"
#include <queue>


// For speed testing but may prove useful otherwise
class MovingAverage {
  public:
      MovingAverage(int period) : period(period), sum(0.0) {}
  
      double update(double new_value) {
          window.push(new_value);
          sum += new_value;
  
          if (window.size() > period) {
              sum -= window.front();
              window.pop();
          }
  
          return window.size() == period ? sum / period : -1.0; // -1.0 means not enough data yet
      }
  
  private:
      int period;
      double sum;
      std::queue<double> window;
};

/**
 * For GPS coord system: https://pros.cs.purdue.edu/v5/tutorials/topical/gps.html
 */

namespace aon {

#if USING_BLACK_ROBOT

int move(double dist);
int turn(double angle);
double metersToInches(double meters);
void grabGoal(const short &delay);
void raceToGoal(double dist);
void driveIntoRing(const short &color);
void pickUpRing(int delay);
void scoreRing(int delay);
inline double metersToInches(double meters);
void discardDisk();
void dropGoal();
void moveIndexer(bool extend);
void enableGate();
void motionProfile(double dist);
void turretRotationAbsolute(double targetAngle);
void turretTrackRestricted(const short &color);


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
 * \brief Determines the speed of the robot given drivetrain motors' RPM
 *
 * \param RPM The RPM for which to calculate the velocity (default current RPM)
 *
 * \returns The speed in \b in/s at which the robot would move at the given RPM
 *
 * \note Test the accuracy precision of the `getActualVelocity()` method,
 * \note it may be possible to need to use `get_velocity()` from `pros::Rotation` which uses \b centidegrees.
 * \note The distance units depend on the units used for measuring `DRIVE_WHEEL_DIAMETER`
 */
inline double getSpeed(const double &RPM = (int)driveFull.getActualVelocity()){
  double circumference = DRIVE_WHEEL_DIAMETER * M_PI;
  double RPS = RPM / 60;
  double speed = MOTOR_TO_DRIVE_RATIO * circumference * RPS;
  return speed;
}

/**
 * \brief Calculates time for the robot to reach a given distance
 *
 * \param distance Distance from the robot to the target (remains constant) in \b inches
 * \returns The approximate time necessary to reach the target (overestimation) in \b seconds
 */
inline double getTimetoTarget(const double &distance, const double &RPM = MAX_RPM){
  double time = 4 * distance / getSpeed(RPM);
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
inline double getTimetoTurnRad(const double &radians, const double &RPM = MAX_RPM / 4){
  double arcLength = radians * AVG_DRIVETRAIN_RADIUS; // Of the turn (inches)
  double time = 2 * arcLength / getSpeed(RPM); // Calculated time (seconds)
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
 * \brief Gets the distance between two points in the field
 *
 * \param target The target location
 * \param current The current location
 *
 * \returns The distance between the two points
 */
double findDistance(Vector target, Vector current){
  double distInMeters= (target - current).GetMagnitude();
  return metersToInches(distInMeters);
}

/**
 * \brief Determines the angle needed to be turned in order to face a specific point in the field
 *
 * \param target The point we wish to face
 * \param current Where the robot is now
 *
 * \returns The angle the robot needs to turn in order to face the target location
 *
 * \note The result must be passed into functions such as turn() and MoveTurnPID() as negative because of their convention
 */
double calculateTurn(Vector target, Vector current) {
  // Get and change the heading to the common cartesian plane
  double heading = 90 - gps.get_heading();

  // Limiting the heading to the 0-360 range
  if (heading < 0) heading += 360;
  else if (heading > 360) heading -= 360;
 
  // This number is in respect to the common cartesian plane if odometry position is used
  double toTarget = (target - current).GetDegrees();
 
  // Limiting the the target to the 0-360 range
  if (toTarget < 0) toTarget += 360;
  else if (toTarget >= 360) toTarget -= 360;

  double angle = toTarget - heading; // Calculate the angle to turn
 
  // Limiting the heading to the -180-180 range
  if (angle > 180) angle -= 360;
  else if (angle < -180) angle += 360;

  return angle;
}

/**
 * \brief Calculates the error percentage for an actual given the expected value
 * 
 * \param expected Usually a calculated value
 * \param actual Usually the measured value
 * 
 * \returns The error percentage for the measured value
 */
double getError(double expected, double actual) {
  return ((expected - actual) / expected) * 100;
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
 * \param MAX_REVS The maximum RPM to send to the movement
 */
void MoveDrivePID(PID pid = drivePID, double dist = TILE_WIDTH, const double &MAX_REVS = 100.0) {
  motionProfile(dist);
  return; // These are a temporary override while the PID is tuned correctly
  const int sign = dist / abs(dist); // Getting the direction of the movement
  dist = abs(dist); // Setting the magnitude to positive

  // Define the initialPos using the GPS instead of odometry (later should be both)
  // aon::Vector initialPos = getGPSPos();
  pid.Reset();
  aon::Vector initialPos = aon::odometry::GetPosition();

  const double timeLimit = getTimetoTarget(dist, MAX_REVS);
  const double start_time = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - start_time //every time the variable is called it is recalculated automatically

  // while (time < timeLimit) {
  while((aon::odometry::GetPosition() - initialPos).GetMagnitude() < dist){
    aon::odometry::Update();

    double currentDisplacement = (aon::odometry::GetPosition() - initialPos).GetMagnitude();

    double output = pid.Output(dist, currentDisplacement);

    pros::lcd::print(0, "Time Limit %.2f", timeLimit);
    pros::lcd::print(1, "Time: %.2f", time);
    pros::lcd::print(2, "Odometry Displacement %.2f", currentDisplacement);

    driveFull.moveVelocity(sign * std::clamp(output * (int)driveLeft.getGearing(), -MAX_REVS, MAX_REVS));

    pros::delay(10);
  }

  // Stop the motors
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
void MoveTurnPID(PID pid = turnPID, double angle = 90, const double &MAX_REVS = 50.0){
  const int sign = angle/abs(angle); // Getting the direction of the movement
  angle = abs(angle); // Setting the magnitude to positive
  pid.Reset();
  gyroscope.tare(); // .tare() or .reset(true) depending on the time issue
  const double startAngle = gyroscope.get_heading(); // Angle relative to the start
  
  double timeLimit = getTimetoTurnDeg(angle);

  if(sign == -1) { angle = 360.0 - angle + CLOCKWISE_ROTATION_DEGREES_OFFSET; }
  if(sign == 1) { angle -= CLOCKWISE_ROTATION_DEGREES_OFFSET; }

  const double targetAngle = angle;

  const double startTime = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - startTime

  // while(time < 1.7 * timeLimit){
  while(time < 3 * timeLimit){

    double traveledAngle = gyroscope.get_heading() - startAngle;

    double output = std::abs(pid.Output(targetAngle, traveledAngle)); //Use the absolute value of the output because if not, counter-clockwise turning is weird (error)

    pros::lcd::print(0, "Time Limit %.2f", timeLimit);
    pros::lcd::print(1, "Time: %.2f", time);
    pros::lcd::print(2, "Gyroscope Displacement %.2f", traveledAngle);

    // Taking clockwise rotation as positive (to change this just flip the negative on the sign below)
    driveLeft.moveVelocity(sign * std::clamp(output * (int)driveLeft.getGearing(), -MAX_REVS, MAX_REVS));
    driveRight.moveVelocity(-sign * std::clamp(output * (int)driveRight.getGearing(), -MAX_REVS, MAX_REVS));

    pros::delay(10);
  }

  driveLeft.moveVelocity(0);
  driveRight.moveVelocity(0);

  #undef time
}

/**
 * \brief Turns the robot towards a specific direction
 *
 * \param x The x component of the point we wish to face
 * \param y The y component of the point we wish to face
 * 
 * \note Uses coordinate system from GPS in \b meters
*/
void turnToTarget(double x, double y){
  Vector target = Vector().SetPosition(x, y);
  // Determine current position
  Vector current = position();

  // Do the movement
  turn(-calculateTurn(target, current));
}

/**
 * \brief Goes to the target point
 *
 * \param x The x component of the place where we want to go using the gps coordinate system (x, y) both need to be in the range (-1.8, 1.8)
 * \param y The y component of the place where we want to go using the gps coordinate system (x, y) both need to be in the range (-1.8, 1.8)
 *  
 * \note Uses coordinate system from GPS in \b meters
*/
void goToTarget(double x, double y){
  Vector target = Vector().SetPosition(x, y);
  // Determine current position
  Vector current = position();

  // Do the movement
  turn(-calculateTurn(target, current));
  motionProfile(metersToInches(abs((target - current).GetMagnitude())));
}

/**
 * \brief S-graph motion profile
 * 
 * \param dist The distance to be moved
 */
void motionProfile(double dist = TILE_WIDTH){
  const int sign = dist / abs(dist); // Getting the direction of the movement
  dist = abs(dist); // Setting the magnitude to positive

  const double MAX_VELOCITY = MAX_RPM;//(double)driveFull.getGearing(); // (RPM)
  const double MAX_JERK = MAX_ACCEL; //300; // (RPM/s^2)
  double dt = 0.02; // (s)
  double currVelocity = 0;
  double currAccel = 0;
  double traveledDist = 0;
  Vector startPos = aon::odometry::GetPosition();

  double lastTime = pros::micros() / 1E6;

  while(traveledDist < dist){
    traveledDist = (aon::odometry::GetPosition() - startPos).GetMagnitude();
    double remainingDist = dist - traveledDist;
    dt = (pros::micros() / 1E6) - lastTime;
    lastTime = pros::micros() / 1E6;

    // Debugging output to brain
    pros::lcd::print(1, "Traveled: %.2f / %.2f", traveledDist, dist);
    pros::lcd::print(2, "RPM: %.2f, Accel: %.2f", currVelocity, currAccel);
    pros::lcd::print(3, "Remaining: %.2f", remainingDist);
    pros::lcd::print(4, "Calculated Velocity: %.2f", getSpeed(currVelocity));
    pros::lcd::print(5, "Max Velocity: %.2f", getSpeed(MAX_RPM));

    // Acceleration
    // For the condition, consider half the deceleration for accuracy (there is an error of half an inch almost constant when not used, I have to investigate a bit further on that part but if works fine like this)
    if(remainingDist <= getSpeed(currVelocity) * getSpeed(currVelocity) / (2 * getSpeed(MAX_DECEL * .5))){
      // currAccel = std::max(currAccel - (MAX_JERK * dt), 0);
      currAccel = - MAX_DECEL;
    } else {
      currAccel = std::min(currAccel + (MAX_JERK * dt), MAX_ACCEL);
    }

    currVelocity += currAccel * dt;
    currVelocity = std::min(currVelocity,  MAX_VELOCITY);

    driveFull.moveVelocity(sign * currVelocity);

    // traveledDist += getSpeed(currVelocity) * dt; // TODO: Use Odometry
    if(traveledDist >= dist) { break; } // Overshoot prevention

    pros::delay(20);
  }

  driveFull.moveVelocity(0);
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
void moveTilesStraight(double amt = 1) {
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
 * \brief Moves the robot a given distance
 *
 * \param dist The distance to move in \b inches
 */
int move(double dist = TILE_WIDTH)
{
  MoveDrivePID(drivePID, dist, (double)driveFull.getGearing() / 2);
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


// ============================================================================|
//   ____        _       ____             _   _                
//  / ___| _   _| |__   |  _ \ ___  _   _| |_(_)_ __   ___  ___
//  \___ \| | | | '_ \  | |_) / _ \| | | | __| | '_ \ / _ \/ __|
//   ___) | |_| | |_) | |  _ < (_) | |_| | |_| | | | |  __/\__ \
//  |____/ \__,_|_.__/  |_| \_\___/ \__,_|\__|_|_| |_|\___||___/
// ============================================================================|

/**
 * \brief This small subroutine moves the intake such that a ring is scored on the mobile goal being carried
 *
 * \param delay The time in \b milliseconds to leave the intake running
 */
void pickUpRing(int delay = 1000){
  intake.moveVelocity(INTAKE_VELOCITY / .8);
  pros::delay(delay);
  intake.moveVelocity(0);
}

/**
 * \brief This small subroutine moves the rail such that a ring is scored on the mobile goal being carried
 *
 * \param delay The time in \b milliseconds to leave the intake running
 */
void scoreRing(int delay = 1500){
  rail.moveVelocity(INTAKE_VELOCITY);
  pros::delay(delay);
  rail.moveVelocity(0);
}

volatile bool intakeRunning = true;

/**
 * \brief Asynchronous task for activating the intake when a ring is encountered
 */
void intakeScanning(){
  while(true){
    intake.moveVelocity(0);
    if (intakeRunning && distanceSensor.get() <= DISTANCE) {
      pickUpRing();
      driveFull.moveVelocity(0); // may need a mutex
      scoreRing();
    }
    pros::delay(20);
  }
}

/**
 * \brief This subroutine follows an object (in our case a ring) with a given color signature and picks it up
 *
 * \param SIGNATURE The id number of the vision signature of the object to follow and pick up
 *
 * \todo Add time constraint in case a ring is never found
 */
void driveIntoRing(const short &color = COLOR){
  const int TOLERANCE = 20;
  const int VISION_FIELD_CENTER = 315 / 2;
  const int SPEED = 150; // 200 is max
  const int ADJUSTMENT = 30;
  while(true){
    auto object = vision_sensor.get_by_sig(0, color);
    const int OBJ_CENTER = object.x_middle_coord;

    if(object.signature == color){
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

      if(distanceSensor.get() <= DISTANCE){
        driveFull.moveVelocity(100);
        pickUpRing(1000);
        break;
      }
    }
    else {
      driveFull.moveVelocity(SPEED);

      if(distanceSensor.get() <= DISTANCE){
        driveFull.moveVelocity(100);
        pickUpRing(3000);
        break;
      }
    }
  }
  driveFull.moveVelocity(0);
  scoreRing(1500); // Remember to do this after to finish pickup
}

/**
 * \brief This small subroutine grabs a goal (stake)
 *
 * \param delay The amount of time in \b milliseconds you will be moving back (500-600 is quick and works)
 *
 * \warning You must already be very close to the goal and facing away (with the clamp towards it)
 *
 * \details This routine uses timing but ideally there would be a way of knowing when we have the goal within our grasp
 */
void grabGoal(const short &delay = 600){
  driveFull.moveVelocity(-100);
  pros::delay(delay * 5 / 6);
  claw.set_value(true);
  pros::delay(delay * 1/6);
  driveFull.moveVelocity(100);
  pros::delay(delay);
  driveFull.moveVelocity(0);
}

/**
 * \brief Discards disk at beginning of match
 *
 * \note This function is really meant for routines that will focus on enemy rings
 */
void discardDisk(){
  intake.moveVelocity(-INTAKE_VELOCITY);
  pros::delay(1000);
  intake.moveVelocity(0);
}

/**
 * \brief This subroutine moves toward a mobile goal IN REVERSE
 *
 * \param dist This is the absolute value of the distance the mobile goal is from the robot in \b inches
 *
 * \details The function already converts the distance to negative so the robot drives into the goal backwards
 *
 */
void raceToGoal(double dist = 47){
  dist = abs(dist);
  MoveDrivePID(fastPID, -dist, (int)driveFull.getGearing());
  grabGoal(300);
}

/**
 * \brief Drops the goal by releasing the claw
 */
void dropGoal(){
  claw.set_value(false);
}

/**
 * \brief Extends or retracts indexer to later knock down rings
 *
 * \param extend If true, indexer will extend, if false, it will retract
 */
void moveIndexer(bool extend = true){
  indexer.set_value((extend ? 1 : 0) );
}

/**
 *
 * \brief This small subroutine removes the top ring of a stack of two and scores the ring at top. use ONLY when the indexer is at the right side of stack.
 *
 */
void RemoveTop(){
  moveIndexer();
  turn(-45);
  moveIndexer(false);
}

/**
 * \brief Drops the gate from starting position so the robot can grab stuff
 */
void enableGate(){
  gate.moveVelocity(-100);
  pros::delay(250);
  gate.moveVelocity(0);
}

/**
 * \brief Aligns TURRET only to the item with the set `color`
 * 
 * \param color The vision sensor signature id of the object tio which we want to align, defaults to `COLOR`
 */
void turretFollow(const short &color = COLOR){
  const int TOLERANCE = 10; // 20
  const int VISION_FIELD_CENTER = 315 / 2;
  int OBJ_CENTER = 0;
  double position, lastPos;
  
  while(true){
    auto object = vision_sensor.get_by_sig(0, color);
    OBJ_CENTER = object.x_middle_coord;
    double SPEED = turretPID.Output(0, VISION_FIELD_CENTER - OBJ_CENTER);
    position = turretEncoder.get_angle() / 100;
    pros::lcd::print(1, "Position: %.2f", position);
    pros::lcd::print(3, "Last Pos: %.2f, Curr Pos %.2f", lastPos, position);
    pros::lcd::print(4, "Right Limit: %d, Left Limit %d", ORBIT_RIGHT_LIMIT, ORBIT_LEFT_LIMIT);
    pros::lcd::print(5, "Right cond: %d, Left cond %d", lastPos > ORBIT_RIGHT_LIMIT && position <= ORBIT_RIGHT_LIMIT, lastPos < ORBIT_LEFT_LIMIT && position >= ORBIT_LEFT_LIMIT);

    if(object.signature == color){
      if(abs(OBJ_CENTER - VISION_FIELD_CENTER) <= TOLERANCE){
        turret.moveVelocity(0);
        pros::lcd::print(2, "Aligned!");
        break;
      }
      // Limiting to protect hardware
      else if (ORBIT_LIMITED && (ORBIT_LEFT_LIMIT <= position && position <= ORBIT_RIGHT_LIMIT)) {
        turret.moveVelocity(0);
        // turretRotationAbsolute(0);
      }
      else { // Turn Towards Object
        pros::lcd::print(2, "Turning!");
        turret.moveVelocity(SPEED);
      }
    }
    // Dont move if nothing is there
    else {
      turret.moveVelocity(0);
      break;
    }
    lastPos = position;
    pros::delay(10);
  }
  turret.moveVelocity(0);
}

/**
 * \brief Aligns TURRET and DRIVETRAIN to the item with the set `COLOR`
 * 
 * \note Setting `color` to `STAKE` makes the robot turn 180° after alignment
 */
void alignRobotTo(const short &color = COLOR){
  turretFollow(color);
  const int TOLERANCE = 10;
  double difference;
  #define TURRET_ANGLE turretEncoder.get_angle() / 100
  while(abs((TURRET_ANGLE)) > TOLERANCE){
    difference = TURRET_ANGLE < 180 ? TURRET_ANGLE : TURRET_ANGLE - 360;
    pros::lcd::print(2, "Moving!");
    double SPEED = turnPID.Output(0, difference) * 400;
    pros::lcd::print(3, "Speed: %.2f", SPEED);
    driveLeft.moveVelocity(SPEED);
    driveRight.moveVelocity(-SPEED);
    turretFollow(color);
  }
  #undef TURRET_ANGLE
  driveLeft.moveVelocity(0);
  driveRight.moveVelocity(0);
  if(color == STAKE){
    turn(180);
  }
}

/**
 * \brief Aligns TURRET and DRIVETRAIN to the item with the set `COLOR`
 * 
 * \note Setting `color` to `STAKE` makes the robot turn 180°
 */
void grabRing(const short &color = COLOR){
  turretFollow(color);
  const int TOLERANCE = 10;
  const double FORWARD_SPEED = 100;
  double difference;
  #define TURRET_ANGLE turretEncoder.get_angle() / 100
  while(abs((TURRET_ANGLE)) > TOLERANCE){
    difference = TURRET_ANGLE < 180 ? TURRET_ANGLE : TURRET_ANGLE - 360;
    pros::lcd::print(2, "Moving!");
    double SPEED = turnPID.Output(0, difference) * 400;
    pros::lcd::print(3, "Speed: %.2f", SPEED);
    driveLeft.moveVelocity(FORWARD_SPEED + SPEED);
    driveRight.moveVelocity(FORWARD_SPEED -SPEED);
    turretFollow(color);
    // Stop and pick up if close
    if (distanceSensor.get() <= DISTANCE) {
      pickUpRing(3000);
      break;
    }
    driveFull.moveVelocity(0);
  }
  #undef TURRET_ANGLE
}

/**
 * \brief Aligns front of robot and turns around to grab de stake
 * 
 * \param dist The absolute value of the distance that the robot is from the stake when it begins alignment
 */
void dumbGetStake(const double &dist = 8){
  alignRobotTo(STAKE);
  move(-abs(dist));
  grabGoal();
}

/**
 * \brief Rotates the turret a given angle, starting from 0 degrees until desired degrees. (Absolute Rotation)
 * 
 * \param targetAngle Angle in degrees we wish to rotate turret.
 *
 * \details turretEncoder.get_angle() is divided by 100 for scaling purposes.
 */
inline void turretRotationAbsolute(double targetAngle) { 
  while (true) {
    double currentAngle = turretEncoder.get_angle()/100.0; 
    double output = turretPID.Output(targetAngle, currentAngle); 
    turret.moveVelocity(output); 
    pros::delay(10);
  }
  turret.moveVelocity(0);
}


/**
 * \brief Rotates the turret a given angle, starting from wherever it currently is. (Relative Rotation)
 * 
 * \param givenAngle Angle in degrees we wish to rotate turret.
 *
 * \details `turretEncoder.get_angle()` is divided by 100 for scaling purposes.
 */
inline void turretRotationRelative(double givenAngle) { 
  double currentAngle = turretEncoder.get_angle() / 100.0; 
  double initialAngle = turretEncoder.get_angle() / 100.0; 
  double targetAngle = initialAngle + givenAngle; 
  while (true) {
    currentAngle = turretEncoder.get_angle() / 100.0;
    double output = turretPID.Output(targetAngle, currentAngle); 
    turret.moveVelocity(output); 
    pros::delay(10);
  }
  turret.moveVelocity(0);
}

/**
 * \brief Drives forward until a ring hits the distance sensor
 * 
 * \param TIMEOUT The max time the robot will be moving forward in \b seconds (default is 5 sec)
 */
void driveTillPickUp(const double &TIMEOUT = 5){
  const double startTime = pros::micros() / 1E6;
  #define time pros::micros() / 1E6 - startTime
  while(time  < TIMEOUT){
    driveFull.moveVelocity(200);
    if (distanceSensor.get() <= DISTANCE) {
      pickUpRing(1000);
      driveFull.moveVelocity(0);
      scoreRing();
      break;
    }
  }
  #undef time
  driveFull.moveVelocity(0);
}

// ============================================================================
//   _____ ___ ___ _____ ___
//  |_   _| __/ __|_   _/ __|
//    | | | _|\__ \ | | \__ \
//    |_| |___|___/ |_| |___/
//
// ============================================================================

/**
 * \brief Basic Routine to make the robot go in circles around the map to test GPS setup.
 */
void testGPS() {
  aon::goToTarget(.6, -1.2);
  aon::goToTarget(1.2, -.6);
  aon::goToTarget(1.2, .6);
  aon::goToTarget(.6, 1.2);
  aon::goToTarget(-.6, 1.2);
  aon::goToTarget(-1.2, .6);
  aon::goToTarget(-1.2, -.6);
  aon::goToTarget(-.6, -1.2);
  aon::goToTarget(.6, -1.2);
  aon::goToTarget(1.2, -.6);
}

/**
 * \brief Speed calculation test using the distance sensor
 */
void testSpeed(double RPM = (double)driveFull.getGearing()){
  MovingAverage mav(50);
  while(true) {
    driveFull.moveVelocity(RPM);
    double measured = metersToInches(distanceSensor.get_object_velocity());
    double calculated = getSpeed(RPM);
    double error = abs(getError(calculated, measured));
    double avg = mav.update(error);
    pros::lcd::print(1, "RPM: %.2f", RPM);
    pros::lcd::print(2, "Calculated Velocity: %.2f", calculated);
    pros::lcd::print(3, "Measured Velocity: %.2f", measured);
    pros::lcd::print(4, "Error %: %.2f%", avg);
    pros::delay(10);
  }
}

void testOdom(){
  // Motion Profile
  motionProfile(12 * 3);
  pros::delay(1000);
  motionProfile(-12 * 3);
  pros::delay(1000);
  return;

  // PID Forward
  move(12 * 3);
  pros::delay(1000);
  move(-12 * 3);
  pros::delay(1000);

  // PID Rotations
  turn(90);
  pros::delay(1000);
  turn(-90);
  pros::delay(1000);
  turn(45);
  pros::delay(1000);
  turn(45);
  pros::delay(1000);
  turn(-45);
  pros::delay(1000);
  turn(-45);
  pros::delay(1000);
}

void testIndexer(){
  moveIndexer();
  motionProfile(40);
  move(-6);
  turn(180);
  move(-2);
  grabGoal();
}

/**
 * \brief Test to ensure the concurrency is working fine, requires `intakeScanning` to be running in another thread
 */
void testConcurrency(){
  intakeRunning = true;
  int startTime = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - startTime
  while(time < 5){
    driveFull.moveVelocity(100);
    pros::delay(20);
  }
  #undef time
  driveFull.moveVelocity(0);
  intakeRunning = false;
}

void testTurnProfile(double angle = 90){
  const int sign = angle / abs(angle); // Getting the direction of the movement
  angle = abs(angle); // Setting the magnitude to positive
  gyroscope.tare(); // .tare() or .reset(true) depending on the time issue
  
  if(sign == -1) { angle = 360.0 - angle + CLOCKWISE_ROTATION_DEGREES_OFFSET; }
  if(sign == 1) { angle -= CLOCKWISE_ROTATION_DEGREES_OFFSET; }
  
  const double MAX_VELOCITY = MAX_RPM;//(double)driveFull.getGearing(); // (RPM)
  // const double MAX_ACCEL = MAX_VELOCITY * 3; // Try * 4 // (RPM/s)
  // const double MAX_DECEL = 200; //100 has worked in the past // (RPM/s)
  const double MAX_JERK = MAX_ACCEL; //300; // (RPM/s^2)
  double dt = 0.02; // (s)
  double currVelocity = 0;
  double currAccel = 0;
  double traveledAngle = 0;
  double startAngle = gyroscope.get_heading();

  double startTime = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - startTime

  while(traveledAngle < angle){
    traveledAngle = gyroscope.get_heading() - startAngle;
    double remainingAngle = angle - traveledAngle;

    // Debugging output to brain
    pros::lcd::print(1, "Traveled: %.2f / %.2f", traveledAngle, angle);
    pros::lcd::print(2, "RPM: %.2f, Accel: %.2f", currVelocity, currAccel);
    pros::lcd::print(3, "Remaining: %.2f", remainingAngle);
    pros::lcd::print(4, "Calculated Velocity: %.2f", getSpeed(currVelocity));
    pros::lcd::print(5, "Max Velocity: %.2f", getSpeed(MAX_RPM));

    // Acceleration
    if(remainingAngle <= getSpeed(currVelocity) * getSpeed(currVelocity) / (2 * getSpeed(100))){
      // currAccel = std::max(currAccel - (MAX_JERK * dt), 0);
      currAccel = - 100 * 1.5;
    } else {
      currAccel = std::min(currAccel + (MAX_JERK * dt), 200.0);
    }

    currVelocity += currAccel * dt;
    currVelocity = std::min(currVelocity,  MAX_VELOCITY);

    driveLeft.moveVelocity(sign * currVelocity);
    driveRight.moveVelocity(-sign * currVelocity);


    pros::delay(dt * 1000);
  }

  driveFull.moveVelocity(0);
  #undef time
}

void AlignRobotToStake(){//align back of robot to the steak
  // First, align turret to the new target (e.g., yellow stake)
  turretFollow(); 
  const int TOLERANCE = 10;// Degrees of acceptable error for alignment
  #define TURRET_ANGLE turretEncoder.get_angle()/100 // Get turret angle in degrees
  int difference = (TURRET_ANGLE + 180) % 360;

  // Calculate the angle difference needed to rotate the robot's back toward the target.
  // This is done by adding 180° to the turret's angle and normalizing to [-180°, 180°]
  if (difference > 180){
    difference -= 360;
  }
  if (difference < -180){
    difference += 360;
  }

  // Loop until the robot's back is aligned within the specified tolerance
  while(abs(difference) > TOLERANCE){
    //process of aligning back of robot 
    turretFollow(); 
    double SPEED = turnPID.Output(0, difference) * 40;
    driveLeft.moveVelocity(SPEED);
    driveRight.moveVelocity(-SPEED);
    difference = (TURRET_ANGLE + 180) % 360;  // Recalculate the angle difference after turning
    if(difference > 180){
      difference -= 360;
    }
      
    if(difference < -180){
      difference += 360;
    }
      pros::delay(10);
  }
  #undef TURRET_ANGLE

  driveLeft.moveVelocity(0);
  driveRight.moveVelocity(0);
  //back of robot should be aligned 
}


bool continuous_scan(){
  turretRotationAbsolute(180);
  pros::lcd::print(1, "Aligned 180");
  pros::delay(1000);
  while(true){

    turretRotationRelative(90);
    pros::lcd::print(2, "turning 1");
    pros::delay(1000);
    turretRotationRelative(-0);
    pros::lcd::print(2, "turning 2");
    pros::delay(1000);

    auto object= vision_sensor.get_by_sig(0,COLOR);
    
    if(object.signature == COLOR){
      pros::lcd::print(2, "object found");
      pros::delay(20);
      break;
    }
  }
}


void FollowWithTurret(u_int32_t color = COLOR) {
  const int TURRET_TOLERANCE = 5;
  const int BODY_ADJUST = 10;
  const double FORWARD_SPEED = 130;
  #define TURRET_ANGLE turretEncoder.get_angle()/100 // Get turret angle in degrees

  while (true) {
      // Single step turret adjustment (should be non-blocking)
      // turretTrackRestricted(color);
      turretFollow(color);

      int turret_angle = turretEncoder.get_angle() / 100;
      int difference = TURRET_ANGLE < 180 ? TURRET_ANGLE : TURRET_ANGLE - 360;

      // Adjust body if turret deviates too much
      if (abs(difference) > BODY_ADJUST) {
          double turnSpeed = turnPID.Output(0, difference) * 400;
          turnSpeed = std::clamp(turnSpeed, -FORWARD_SPEED, FORWARD_SPEED);
          driveLeft.moveVelocity(FORWARD_SPEED + turnSpeed);
          driveRight.moveVelocity(FORWARD_SPEED - turnSpeed);
      } else {
          driveFull.moveVelocity(FORWARD_SPEED);
      }

      // Stop and pick up if close
      if (distanceSensor.get() <= DISTANCE) {
        pickUpRing(1000);
        driveFull.moveVelocity(0);
        scoreRing();
        break;
      }
      
      // Stop if no objects detected
      if (vision_sensor.get_object_count() == 0) {
        break;
      }

      pros::delay(10);
  }
  #undef TURRET_ANGLE

  driveLeft.moveVelocity(0);
  driveRight.moveVelocity(0);
}


void turretTrackRestricted(const short &color = COLOR) {
  const int TOLERANCE = 20;
  const int VISION_FIELD_CENTER = 315 / 2;
  const int BOUND_1 = 270; // max right
  const int BOUND_2 = 120; // max left
  static PID turretPID = PID(0.5, 0, 0);

  auto object = vision_sensor.get_by_sig(0, color);

  // If no valid object found
  if (object.signature != color) {
      turret.moveVelocity(0);
      //pros::lcd::print(2, "No Object");
      return;
  }

  int OBJ_CENTER = object.x_middle_coord;
  int turret_angle = turretEncoder.get_angle() / 100;
  double SPEED = turretPID.Output(0, VISION_FIELD_CENTER - OBJ_CENTER);

  //pros::lcd::print(1, "Turret Pos: %d", turret_angle);
  //pros::lcd::print(3, "Center Diff: %d", VISION_FIELD_CENTER - OBJ_CENTER);

  // If already aligned
  if (abs(OBJ_CENTER - VISION_FIELD_CENTER) <= TOLERANCE) {
      turret.moveVelocity(0);
      //pros::lcd::print(2, "Turret Aligned");
      return;
  }

  // Predict next angle
  int projected_angle = turret_angle + (SPEED > 0 ? 1 : -1) * 5;

  if (projected_angle >= BOUND_2 && projected_angle <= BOUND_1) {
      turret.moveVelocity(SPEED);
      //pros::lcd::print(2, "Turret Turning");
  } else {
      turret.moveVelocity(0);
      //pros::lcd::print(2, "Turret Out of Bounds");
  }
}




// ============================================================================|
//   ___  ___  _   _ _____ ___ _  _ ___ ___                                    |
//  | _ \/ _ \| | | |_   _|_ _| \| | __/ __|                                   |
//  |   / (_) | |_| | | |  | || .` | _|\__ \                                   |
//  |_|_\\___/ \___/  |_| |___|_|\_|___|___/                                   |
//                                                                             |
// ============================================================================|

/**
 * \brief This is a safety routine to at least grab one goal and score on it
 */
void quickMiddleScore(){
  // goToTarget(.3, 1.2);
  // turnToTarget(0.6, 1.2);
  move(-3);
  grabGoal();
  scoreRing();
  move(10);
 
}

/**
 * \brief This routine is if WE ARE RED and want to grab RED RINGS
 *
 * \note Designed for being in the third quadrant
 * \note Starting Position (-0.34, -0.82) \b m facing towards 296.86 \b deg
 *
 * \author Kevin Gomez
*/
int RedRingsRoutine(){
  // Secure and score the first ring in the middle stake
  raceToGoal();
  move(6);
  scoreRing();
  enableGate();

  // Get the next ring in our side
  turnToTarget(-.6, -1.2);
  move(6);
  RemoveTop();
  move(-6);
  alignRobotTo(COLOR);
  driveTillPickUp();

  // Get the last ring in that line
  move(-6);
  turnToTarget(-1.2, -1.2);
  move(6);
  driveTillPickUp();

  // Bring down the 4 stack
 
  return 1;
}

/**
 * \brief This routine is if WE ARE BLUE and want to grab BLUE RINGS
 *
 * \note Designed for being in the first quadrant
 * \note Starting Position (0.34, 0.82) \b m facing towards 116.86 \b deg
 *
 * \author Kevin Gomez
 */
int BlueRingsRoutine(){
  // Secure and score the first ring in the middle stake
  raceToGoal();
  move(6);
  scoreRing();
  enableGate();
  
  // Get the next ring in our side
  turnToTarget(.6, 1.2);
  move(6);
  RemoveTop();
  move(-6);
  alignRobotTo(COLOR);
  driveTillPickUp();
  
  // Get the last ring in that line
  move(-6);
  turnToTarget(1.2, 1.2);
  move(6);
  driveTillPickUp();
  
  // Bring down the 4 stack
  
  return 1;
}

void scoreWithIndexer(){
  
}

/**
  WILL CLEAR POSITIVE SIDE JUST TO BE SURE

  LOOKING AT THE POSITIVE SIDE OF OUR SIDE

  TRY TO PUT IT 4 INCHES AWAY AS BEST AS POSSIBLE
  */

int BlueRingsRoutine_JorgeGuz(){
  // Go into the esquina
  move(4);
  pickUpRing(1500);
  move(-4);
  move(4);
  pickUpRing(1500);
  move(-4);
  move(4);
  pickUpRing(1500);
  move(-4);
  
}

/**
  Will take the closest to us, in our side and take th rings that are below

  START IN THE POSITIVE SIDE OF THE FIELD, LOOKING AT THE STAKE WHEN WE ARE RED.

  RED-NEGATIVE SIDE
*/
int safeRingRoutine() {
  dumbGetStake(8); // 8 inches from stake
  turnToTarget(-1.2,-1.2);
  goToTarget(-1.2,-1.2);
  FollowWithTurret(RED);
  turnToTarget(-1.5,0);
  goToTarget(-1.5,0);
  FollowWithTurret(RED);
  turnToTarget(1.2,-1.2);
  goToTarget(1.2,-1.2);
  FollowWithTurret(RED);
}

int safeRingRoutine2() {
  dumbGetStake(6); //6 inches from stake
  turnToTarget(-1.2,-1.2);
  goToTarget(-1.2,-1.2);
  FollowWithTurret(RED);
  turnToTarget(-1.5,0);
  goToTarget(-1.5,0);
  FollowWithTurret(RED);
  turnToTarget(-1.2,1.2);
  goToTarget(-1.2,1.2);
  FollowWithTurret(RED);
}





/**
 * \brief This routine is if WE ARE BLUE and want to grab BLUE RINGS
 *
 * \author Jorge Luis
*/    

int BlueRingsRoutineJorgeLuna() {
  /*
    go for negative side mobile goal, score rings, and prepare for go enemy double side
  */
  // go to the side mobile goal
  raceToGoal();
  move(6);
  scoreRing(2000);
  enableGate();

  // go to ring on the bottom
  goToTarget(1.2, -0.55);
  RemoveTop();
  driveIntoRing(COLOR);

  // then the one below that one
  goToTarget(1.2, -1.1);
  driveIntoRing(COLOR);

  // drive into the corner and try to grab the rings
  goToTarget(1.7, -1.7);
  RemoveTop();
  driveIntoRing(COLOR);
  turnToTarget(1.7, -1.7);
  RemoveTop();
  driveIntoRing(COLOR);

  move(-6);
  turnToTarget(1.8, 1.8);
}

/**
 * \author Jorge L
 */
int SkillsBlackBotJorge(){
  // grab skate in the middle bottom
  raceToGoal(33.4066376232);
  // grab 0, -1.2
  // grab 0.6, -1.2
  // turn to -1.4, 0
  // grab -1.4, 0
  // turn to -0.6, -0.6
  // go closer
  // grab -0.6, -0.6
  // turn to -1.2, -1.2
  // grab it
  // grab -1.8, -1.8
  // turn 180 
  // let stake at the esquina
  
  // go to 0.6, -0.6
  // grab stake
  // grab rings in the middle
  // let stake
  // go to -.6, -0.6
  // grab stake most right
  // take ring 1.5, 0
  // take ring 1.2, -0.6
  // take ring 0.6, -1.2
  // take ring 1.2, -1.2
  // take ring -1.8, 1.8
  // if we suppose all the red rings are as points
    // take blue ring 1.8, -1.8
  // put stake in 1.8, -1.8
}

/**
 * \author Kevin
 * 
 * \note Starts with claw in (-1.2, -6) facing bottom-most goal
 */
int SkillsBlackBotKevin(){
  // Grab bottom-most goal
  raceToGoal(33.4066376232);
  
  // Grab ring in (-1.2, -1.2)
  turnToTarget(-1.2, -1.2);
  FollowWithTurret();
  move(-12);

  // Grab ring in (-1.8, -1.8)
  turnToTarget(-1.8, -1.8);
  move(6);
  FollowWithTurret();
  move(-12);

  // Grab ring in (-.6, -.6)
  goToTarget(-.9, -.9);
  FollowWithTurret();
  move(-24);

  // Grab ring in (0, -1.2)
  turnToTarget(0, -1.2);
  
  
}



#else

int greenBotRedSide(){
  move(-6);
  grabGoal();
  scoreRing();
  enableGate();

  turnToTarget(-1.2,1.2);
  moveTilesStraight(1.3);
  driveTillPickUp();

  turnToTarget(-0.6,1.2);
  moveTilesStraight(1);
  driveTillPickUp();

  turnToTarget(-1.5,0);
  moveTilesStraight(1.3);
  driveTillPickUp();
}

void simple_Auto_Red(){
    move(-.5);
    grabGoal();
    scoreRing();
    enableGate();
  
    turnToTarget(-1.2,1.2);
    moveTilesStraight(1.3);
    driveIntoRing(COLOR);// change
  
    turnToTarget(-0.6,1.2);
    moveTilesStraight(1);
    driveIntoRing(COLOR);//change
  
    turnToTarget(-1.5,0);
    moveTilesStraight(1.3);
    driveIntoRing(COLOR);//change
  
}

void Auto_with_indexer(){
  moveTilesStraight(1.5);
  moveIndexer();
  moveTilesStraight(-.5);
  turn(180)
  grabGoal();
  scoreRing();
  enableGate();

  turnToTarget(-0.6,1.2);
  moveTilesStraight(1);
  driveIntoRing(COLOR); //channge 

  turnToTarget(-1.2,1.2);
  moveTilesStraight(1.3);
  driveIntoRing(COLOR);// change

  turnToTarget(-1.5,0);
  moveTilesStraight(1.3);
  driveIntoRing(COLOR);//change


}


/**
 * \author Solimar
 */
int SkillsGreenBotSoli(){
  //First Grab Nearest stake 
  move(-1.3);
  grabGoal();
  
  //Then attempt to grab the red rings towards the corner 
  turnToTarget(-1.8, 1.8);
  driveIntoRing(COLOR);
  driveIntoRing(COLOR);
  
  //turn towards remaining going up
  //turn towards red on the line
  //drop steak
  goToTarget(-1.5, -0.3);
  turnToTarget(-1.2, 0.0);
  FollowWithTurret(RED);
  dropGoal();
  
  //pick up second steak 
  turnToTarget(0.6, 0.6);
  turn(180);
  moveTilesDiag(-1);
  grabGoal();
  //pick up reds around whilst also tumbando los stacks red-blue
  turnToTarget(1.8, 1.8);
  driveIntoRing(COLOR);
  driveIntoRing(COLOR);
  
  //turn towards remaining going up
  //turn towards red on the line
  //drop steak
  goToTarget(1.5, -0.3);
  turnToTarget(-1.2, 0.0);
  FollowWithTurret(RED);
  dropGoal();
}

/**
 * \author Jorge G
 */
int SkillsGreenBotJorge(){
  raceToGoal(50); //preg a kev
  turnToTarget(0, -1.2);
  goToTarget(0, -1.2);
  FollowWithTurret(RED);
  //First ring ^
  turnToTarget(-0.6, 0.6);
  goToTarget(-0.6, 0.6);
  FollowWithTurret(RED);
  // Second ring ^
  turnToTarget(-1.2, 1.2);
  goToTarget(-1.2, 1.2);
  FollowWithTurret(RED);
  // Third Ring ^
  turnToTarget(-1.8, 1.8);
  goToTarget(-1.8, 1.8);
  FollowWithTurret(RED); 
  // Fourth Ring ^
  turnToTarget(0, 1.2);
  goToTarget(0, 1.2);
  FollowWithTurret(RED); 
  // Fifth Ring ^
  dropGoal();
  // release stake
  turnToTarget(0.6, 0.6);
  move(12); // measure
  dumbGetStake(10); //measure
  // grab stake at (0.6, 0.6)
  turnToTarget(0, 1.5);
  goToTarget(0, 1.5);
  FollowWithTurret(RED); 
  // 2nd stake first ring
  turnToTarget(0.6, 1.2);
  goToTarget(0.6, 1.2);
  FollowWithTurret(RED);   
  // Second Ring ^
  turnToTarget(1.2, 1.2);
  goToTarget(1.2, 1.2);
  FollowWithTurret(RED);  
  // Third Ring ^
  turnToTarget(1.2, 0.6);
  goToTarget(1.2, 0.6);
  FollowWithTurret(RED);  
  // Fourth Ring ^
  turnToTarget(1.8, 1.8);
  goToTarget(1.8, 1.8);
  FollowWithTurret(RED); 
  // Fifth Ring ^
  move(-3);
  FollowWithTurret(BLUE);
  dropGoal();
}
#endif

};  // namespace aon


