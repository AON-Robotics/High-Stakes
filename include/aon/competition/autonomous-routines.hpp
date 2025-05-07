#pragma once

#include <cmath>
#include <algorithm>
#include "../constants.hpp"
#include "../globals.hpp"
#include "../sensing/odometry.hpp"
#include "../controls/pid/pid.hpp"
#include "../controls/holonomic-motion.hpp"

/**
 * For GPS coord system: https://pros.cs.purdue.edu/v5/tutorials/topical/gps.html
 */

namespace aon {

double initial_pos_x;
double initial_pos_y;
double initial_heading;

/**
 * \brief Returns position of the robot in the field
 *
 * \returns The GPS coordinates as a `Vector`
 */
Vector POSITION(){
  STOP();
  pros::delay(2000);
  pros::c::gps_status_s_t status = gps.get_status();
  Vector current = Vector().SetPosition(status.x, status.y);

  return current;
}

#if USING_15_INCH_ROBOT

int move(double dist);
int turn(double angle);
double metersToInches(double meters);
void grabGoal(int delay);
void raceToGoal(double dist);
void driveIntoRing(const int SIGNATURE);
void pickUpRing(int delay);
void scoreRing(int delay);
inline double metersToInches(double meters);
void discardDisk();
void dropGoal();
void moveIndexer(bool extend);
void enableGate();



/**
 * \brief Rotates the turret a given angle, starting from 0 degrees until desired degrees. (Absolute Rotation)
 * 
 * \param targetAngle Angle in degrees we wish to rotate turret.
 *
 * \details turretEncoder.get_angle() is divided by 100 for scaling purposes.
 */
inline void turretRotationAbsolute(double targetAngle) { 
  PID turretPID = PID(0.5, 0, 0); 
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
 * \details turretEncoder.get_angle() is divided by 100 for scaling purposes.
 */
inline void turretRotationRelative(double givenAngle) { 
  PID turretPID = PID(0.5, 0, 0); 
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
 * \brief Resets odometry and gyro for error accumulation cleanse
 *
 * \warning This function takes three (3) \b seconds to complete
 */
int initialReset(bool gyro = true)
{
  odometry::ResetInitial();
 
  // 3 seconds
  // gyroscope.reset(gyro);
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
 * \note it may be possible to need to use `get_velocity()` from `encoder` which uses \b centidegrees.
 * \note The distance units depend on the units used for measuring `DRIVE_WHEEL_DIAMETER`
 */
inline double getSpeed(const double &RPM = (int)driveFull.getActualVelocity()){
  double circumference = DRIVE_WHEEL_DIAMETER * M_PI;
  double RPS = RPM / 60;
  double speed = circumference * RPS;
  return speed;
}

/**
 * \brief Calculates time for the robot to reach a given distance
 *
 * \param distance Distance from the robot to the target (remains constant) in \b inches
 * \returns The approximate time necessary to reach the target (overestimation) in \b seconds
 */
inline double getTimetoTarget(const double &distance){
  double time = 2 * distance / MAX_SPEED;
  return time;
}

inline double getTimeToTargetFast(const double &distance){
  return getTimetoTarget(distance) / 2;
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
  double time = 1.8 * arcLength / velocity; // Calculated time (seconds)
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
 * \brief Gets the distance between two points in the field
 *
 * \param target The target location
 * \param current The current location
 *
 * \returns The distance between the two points
 */
double findDistance(Vector target, Vector current){
  double Dist_meters= (target - current).GetMagnitude();
  return metersToInches(Dist_meters);
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
 * \todo Test thoroughly
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
void MoveDrivePID(PID pid = drivePID, double dist = TILE_WIDTH, const double MAX_REVS = 30.0) {
  const int sign = dist / abs(dist); // Getting the direction of the movement
  dist = abs(dist); // Setting the magnitude to positive

  // Define the initialPos using the GPS instead of odometry (later should be both)
  // aon::Vector initialPos = getGPSPos();
  pid.Reset();
  aon::Vector initialPos = aon::odometry::GetPosition();

  const double timeLimit = MAX_REVS == (int)driveFull.getGearing() ? getTimeToTargetFast(dist) : getTimetoTarget(dist);
  const double start_time = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - start_time //every time the variable is called it is recalculated automatically

  while (time < timeLimit) {
    aon::odometry::Update();

    double currentDisplacement = (aon::odometry::GetPosition() - initialPos).GetMagnitude();

    double output = pid.Output(dist, currentDisplacement);

    pros::lcd::print(0, "%f", currentDisplacement);

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
void MoveTurnPID(PID pid = turnPID, double angle = 90, const double MAX_REVS = 50.0){
  const int sign = angle/abs(angle); // Getting the direction of the movement
  angle = abs(angle); // Setting the magnitude to positive
  pid.Reset();
  gyroscope.tare(); // .tare() or .reset(true) depending on the time issue
  aon::odometry::gyro_data.prevDegrees = 0; // Restart gyro data everytime gyro is restart
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
    driveLeft.moveVelocity(sign * std::clamp(output * (int)driveLeft.getGearing(), -MAX_REVS, MAX_REVS));
    driveRight.moveVelocity(-sign * std::clamp(output * (int)driveRight.getGearing(), -MAX_REVS, MAX_REVS));
    
    pros::delay(10);
  }
  std::cout << "Ends rotation function.\nlkfpaosurfkqnadipfuhasoidnf;ipasdhfo9uasd\n;asohfcpoiaiw coiphawiuehfipuawhifnd\n;aliowhyefiuanijdfchasiuhefcipuawhiufasdldifyqipenc\naijhfdpianoifphawoiwpenfipawhc9pawe\nolahyfpio awiupghecijawneicuhaslijefch9p;a9enc\n;aoiweyrfa;wcjniahef9phacn\npaiuisdhcaiwchiaewrbfuhawbce8ohawibfpiaw\nalishjefiuauwencilasgfciawboliecbasilfh\nlifhaelkncias";

  // aon::odometry::UpdateTurn(gyroscope.get_heading());

  driveLeft.moveVelocity(0);
  driveRight.moveVelocity(0);

  #undef time
}

/**
 * \brief Turns the robot towards a specific direction
 *
 * \param x The x component of the point we wish to face
 * \param y The y component of the point we wish to face
*/
void turnToTarget(double x, double y){
  Vector target = Vector().SetPosition(x, y);
  // Determine current position
  Vector current = POSITION();

  // Do the movement
  turn(-calculateTurn(target, current));
}

/**
 * \brief Goes to the target point
 *
 * \param x The x component of the place where we want to go using the gps coordinate system (x, y) both needto be in the range (-1.8, 1.8)
 * \param y The y component of the place where we want to go using the gps coordinate system (x, y) both needto be in the range (-1.8, 1.8)
 *  
*/
void goToTarget(double x, double y){
  Vector target = Vector().SetPosition(x, y);
  // Determine current position
  Vector current = POSITION();

  // Do the movement
  turn(-calculateTurn(target, current));
  move(metersToInches(abs((target - current).GetMagnitude())));
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
  intake.moveVelocity(INTAKE_VELOCITY);
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
  const int ADJUSTMENT = 30;
  const int DISTANCE = 120;
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

    if(main_controller.get_digital(DIGITAL_B)){ // Safety During testing
      driveFull.moveVelocity(0);
      intake.moveVelocity(0);
      return;
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
void grabGoal(int delay = 600){
  driveFull.moveVelocity(-100);
  pros::delay(delay * 5 / 6);
  piston.set_value(true);
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
void raceToGoal(double dist = 40){
  dist = abs(dist);
  MoveDrivePID(fastPID, -dist, (int)driveFull.getGearing());
  grabGoal(300);
}

/**
 * \brief Drops the goal by releasing the piston
 */
void dropGoal(){
  piston.set_value(false);
}

/**
 * \brief Extends or retracts indexer to later knock down rings
 *
 * \param extend If true, indexer will extend, if false, it will retract
 */
void moveIndexer(bool extend = true){
  indexer.moveVelocity((extend ? -1 : 1) * (int)indexer.getGearing());
  pros::delay(900);
  indexer.moveVelocity(0);
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

void testMotionProfile(const double dist = TILE_WIDTH){
  const double MAX_VELOCITY = (double)driveFull.getGearing(); // (RPM)
  const double MAX_ACCEL = 100; // (RPM/s)
  const double MAX_JERK = 300; // (RPM/s^2)
  double currVelocity = 0;
  double currAccel = 0;
  double traveledDist = 0;
  // Vector startPos = aon::odometry::GetPosition();
  double dt = 0.02; // (s)

  double startTime = pros::micros() / 1E6;
  double secs = 5;
  #define time (pros::micros() / 1E6) - startTime

  while(traveledDist < dist){
    // aon::odometry::Update();
    // traveledDist = (aon::odometry::GetPosition() - startPos).GetMagnitude();
    double remainingDist = dist - traveledDist;

    // Debugging output to brain
    pros::lcd::print(1, "Traveled: %.2f / %.2f", traveledDist, dist);
    pros::lcd::print(2, "Velocity: %.2f, Accel: %.2f", currVelocity, currAccel);
    pros::lcd::print(3, "Remaining: %.2f", remainingDist);

    // Acceleration
    if(remainingDist <= currVelocity * currVelocity / (2 * MAX_ACCEL)){
      // currAccel = std::max(currAccel - (MAX_JERK * dt), 0);
      currAccel = - MAX_ACCEL;
    } else {
      currAccel = std::min(currAccel + (MAX_JERK * dt), MAX_ACCEL);
    }

    currVelocity += currAccel * dt;
    currVelocity = std::min(currVelocity,  MAX_VELOCITY);

    driveFull.moveVelocity(currVelocity);

    traveledDist += currVelocity * dt;

    pros::delay(dt * 1000);
  }

  driveFull.moveVelocity(0);
  testEndpoint();
  #undef time
}

void turretFollow(){
  const int TOLERANCE = 20;
  const int VISION_FIELD_CENTER = 315 / 2;
  int OBJ_CENTER = 0;
  PID turretPID = PID(.5, 0, 0);
  // while(abs(OBJ_CENTER - VISION_FIELD_CENTER) <= TOLERANCE){
  while(true){
    auto object = vision_sensor.get_by_sig(0, COLOR);
    OBJ_CENTER = object.x_middle_coord;
    double SPEED = turretPID.Output(0, VISION_FIELD_CENTER - OBJ_CENTER);
    pros::lcd::print(1, "Position: %.2d", turretEncoder.get_angle() / 100);

    if(object.signature == COLOR){
      if(abs(OBJ_CENTER - VISION_FIELD_CENTER) <= TOLERANCE){
        turret.moveVelocity(0);
        pros::lcd::print(2, "Aligned!");
        break;
      }
      else { // Turn Towards Object
        pros::lcd::print(2, "Turning!");
        turret.moveVelocity(SPEED);
      }
    }
    pros::delay(10);
  }
  turret.moveVelocity(0);
}

void alignRobotToDisk(){
  turretFollow();
  const int TOLERANCE = 10;
  int difference = 0;
  #define TURRET_ANGLE turretEncoder.get_angle() / 100
  while(abs((TURRET_ANGLE)) > TOLERANCE){
    difference = TURRET_ANGLE < 180 ? TURRET_ANGLE : TURRET_ANGLE - 360;
    pros::lcd::print(2, "Moving!");
    double SPEED = turnPID.Output(0, difference) * 40;
    driveLeft.moveVelocity(SPEED);
    driveRight.moveVelocity(-SPEED);
    turretFollow();
  }
  driveLeft.moveVelocity(0);
  driveRight.moveVelocity(0);
}


// ============================================================================|
//   ___  ___  _   _ _____ ___ _  _ ___ ___                                    |
//  | _ \/ _ \| | | |_   _|_ _| \| | __/ __|                                   |
//  |   / (_) | |_| | | |  | || .` | _|\__ \                                   |
//  |_|_\\___/ \___/  |_| |___|_|\_|___|___/                                   |
//                                                                             |
// ============================================================================|

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
  dropGoal();
  enableGate();

  // Get second goal
  goToTarget(-0.9, -0.9);
  goToTarget(-0.9, -0.3);
  turnToTarget(-0.6, -0.6);
  move(-10);
  grabGoal();
  goToTarget(-1.2, -0.6);
  driveIntoRing(COLOR);

  moveTilesStraight(-1);
  goToTarget(-.7, -1.2);
  // Extend indexer and knock down red ring close to intially picked up stake to pick it up
  moveIndexer();
  turn90(-1);
  turn(10);
  moveTilesStraight(-1);

  // Try to pick it up
  driveIntoRing(COLOR);
  moveIndexer(false);

  // Pickup red ring in front of alliance stake
  goToTarget(-1.2, -.3);
  driveIntoRing(COLOR);

  // Try to knock down corner rings
  goToTarget(-1.8, -1.2);
  moveIndexer();
  turn90(-1);
 
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
  dropGoal();
  enableGate();

  // Get second goal
  goToTarget(0.9, 0.9);
  goToTarget(0.9, 0.3);
  turnToTarget(0.6, 0.6);
  move(-10);
  grabGoal();
  goToTarget(1.2, 0.6);
  driveIntoRing(COLOR);

  moveTilesStraight(-1);
  goToTarget(.7, 1.2);
  // Extend indexer and knock down blue ring close to intially picked up stake to pick it up
  moveIndexer();
  turn90(-1);
  turn(10);
  moveTilesStraight(-1);

  // Try to pick it up
  driveIntoRing(COLOR);
  moveIndexer(false);

  // Pickup blue ring in front of alliance stake
  goToTarget(1.2, .3);
  driveIntoRing(COLOR);

  // Try to knock down corner rings
  goToTarget(1.8, 1.2);
  moveIndexer();
  turn90(-1);

  return 1;
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
 * \brief This is a safety routine to at least grab one goal and score on it
 */
void quickMiddleScore(){
  goToTarget(.3, 1.2);
  turnToTarget(0.6, 1.2);
  move(-3);
  grabGoal();
  scoreRing();
  move(10);
  turn(360 * 10);
 
}

#else

int move(double dist);
int turn(double angle);
double metersToInches(double meters);
void grabGoal(int delay);
void raceToGoal(double dist);
void driveIntoRing(const int SIGNATURE);
void pickUpRing(int delay);
void scoreRing(int delay);
void dropGoal();
void moveIndexer(bool extend);
void enableGate();


/**
 * \brief Resets odometry and gyro for error accumulation cleanse
 *
 * \warning This function takes three (3) \b seconds to complete
 */
int initialReset(bool gyro = true)
{
  odometry::ResetInitial();
  drivePID.Reset();
  turnPID.Reset();
 
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

inline double getTimeToTargetFast(const double &distance){
  return getTimetoTarget(distance) / 2;
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
 * \brief Gets the distance between two points in the field
 *
 * \param target The target location
 * \param current The current location
 *
 * \returns The distance between the two points
 */
double findDistance(Vector target, Vector current){
  double Dist_meters= (target - current).GetMagnitude();
  return metersToInches(Dist_meters);
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
 * \todo Test thoroughly
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
void MoveDrivePID(PID pid = drivePID, double dist = TILE_WIDTH, const double MAX_REVS = 100.0) {
  const int sign = dist / abs(dist); // Getting the direction of the movement
  dist = abs(dist); // Setting the magnitude to positive

  // Define the initialPos using the GPS instead of odometry (later should be both)
  // aon::Vector initialPos = getGPSPos();
  pid.Reset();
  aon::Vector initialPos = aon::odometry::GetPosition();

  const double timeLimit = MAX_REVS == (int)driveFull.getGearing() ? getTimeToTargetFast(dist) : getTimetoTarget(dist);
  const double start_time = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - start_time //every time the variable is called it is recalculated automatically

  while (time < timeLimit) {
    aon::odometry::Update();

    double currentDisplacement = (aon::odometry::GetPosition() - initialPos).GetMagnitude();

    double output = pid.Output(dist, currentDisplacement);

    pros::lcd::print(0, "%f", currentDisplacement);

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
void MoveTurnPID(PID pid = turnPID, double angle = 90, const double MAX_REVS = 50.0){
  const int sign = angle/abs(angle); // Getting the direction of the movement
  angle = abs(angle); // Setting the magnitude to positive
  pid.Reset();
  gyroscope.tare(); // .tare() or .reset(true) depending on the time issue
  aon::odometry::gyro_data.prevDegrees = 0;
  
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
*/
void turnToTarget(double x, double y){
  Vector target = Vector().SetPosition(x, y);
  // Determine current position
  Vector current = POSITION();

  // Do the movement
  turn(-calculateTurn(target, current));
}

/**
 * \brief Goes to the target point
 *
 * \param x The x component of the place where we want to go using the gps coordinate system (x, y) both needto be in the range (-1.8, 1.8)
 * \param y The y component of the place where we want to go using the gps coordinate system (x, y) both needto be in the range (-1.8, 1.8)
 *  
*/
void goToTarget(double x, double y){
  Vector target = Vector().SetPosition(x, y);
  // Determine current position
  Vector current = POSITION();

  // Do the movement
  turn(-calculateTurn(target, current));
  move(metersToInches(abs((target - current).GetMagnitude())));
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
  intake.moveVelocity(INTAKE_VELOCITY);
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
  const int ADJUSTMENT = 30;
  const int DISTANCE = 120;
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
void grabGoal(int delay = 600){
  driveFull.moveVelocity(-100);
  pros::delay(delay * 5 / 6);
  piston.set_value(true);
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
 * \todo Change the internal move() function to directly use the MoveDrivePid() function with a specific PID and speed
*/
void raceToGoal(double dist = 40){
  dist = abs(dist);
  MoveDrivePID(fastPID, -dist, (int)driveFull.getGearing());
  grabGoal(300);
}

/**
 * \brief Drops the goal by releasing the piston
 */
void dropGoal(){
  piston.set_value(false);
}

/**
 * \brief Extends or retracts indexer to later knock down rings
 *
 * \param extend If true, indexer will extend, if false, it will retract
 */
void moveIndexer(bool extend = true){
  indexer.moveVelocity((extend ? -1 : 1) * (int)indexer.getGearing());
  pros::delay(900);
  indexer.moveVelocity(0);
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


// ============================================================================
//   _____ ___ ___ _____ ___
//  |_   _| __/ __|_   _/ __|
//    | | | _|\__ \ | | \__ \
//    |_| |___|___/ |_| |___/
//
// ============================================================================

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

// ============================================================================|
//   ___  ___  _   _ _____ ___ _  _ ___ ___                                    |
//  | _ \/ _ \| | | |_   _|_ _| \| | __/ __|                                   |
//  |   / (_) | |_| | | |  | || .` | _|\__ \                                   |
//  |_|_\\___/ \___/  |_| |___|_|\_|___|___/                                   |
//                                                                             |
// ============================================================================|

/**
 * \brief This routine is if WE ARE BLUE and want to grab BLUE RINGS
 *
 * \author Solimar Cruz
*/              
int BlueRingsRoutine(){
  // This routine focuses on team rings (no duh)
  // Rush to one of the side mobile goals (the one on the side of the double points) and secure it on team side
  // Starting point at (2.5, 1.5) facing 90 degrees (the blue area)
  // Rush to one of the side mobile goals (the one on the side of the negative points) and secure it on team side
  //scores one ring onto side mobile goal and drops it
  raceToGoal(45);
  move(9);
  scoreRing(2000);
  dropGoal();
  enableGate();

  // Goes to middle mobile goal to secure it
  goToTarget(0.3, -0.3);
  turnToTarget(0.6, -0.6);
  move(-3);
  grabGoal();

  goToTarget(0.75,-1.05 );
  turnToTarget(1.2,-1.2);
  driveIntoRing(COLOR);
  move(10);


  //knock down stacks
  goToTarget(0.9,-0.9);
  turnToTarget(1.8,-1.8);
  moveIndexer();
  moveTilesStraight(1);
  turn(10);
  //start collecting red rings

  driveIntoRing(COLOR);
  //incase no more rings are found of RED then turn to find
  turn(30); //??
  // Stack team rings on mobile goal with remaining time
  return 1;
}

/**
 * \brief This routine is if WE ARE BLUE and want to grab BLUE RINGS on positive side.
 *
 * \author Jorge Guzmán
*/              
int BlueRingsRoutineJorgeGuzman(){
  raceToGoal(40);
  scoreRing();
  enableGate();

  turnToTarget(0.3, -1.050);
  goToTarget(0.3, -1.050);
  RemoveTop();
  driveIntoRing(1500);
  turnToTarget(1.050, -1.050);
  goToTarget(1.050, -1.050);
  driveIntoRing(1500);
  return 1;
}

/**
 * \brief This routine is if WE ARE RED and want to grab BLUE RINGS
 *
 * \author Solimar Cruz
*/
int RedRingsRoutine(){
  // Rush to one of the side mobile goals (the one on the side of the negative points) and secure it on team side
  //scores one ring onto side mobile goal and drops it
  raceToGoal(40);
  move(8);
  scoreRing(2000);
  dropGoal();
  enableGate();

  // Goes to middle mobile goal to secure it
  goToTarget(-0.3, 0.3);
  turnToTarget(-0.6, 0.6);
  move(-3);
  grabGoal();//grabs middle goal

  goToTarget(-0.75,1.05 );
  turnToTarget(-1.2,1.2);
  driveIntoRing(COLOR);
  move(10);
//knock down stacks
  goToTarget(-0.9,0.9);
  turnToTarget(-1.8,1.8);
  moveIndexer();
  moveTilesStraight(1);
  turn(10);
  //start collecting red rings
  driveIntoRing(COLOR);
  //incase no more rings are found of RED then turn to find
  turn(30); //??
  // Stack team rings on mobile goal with remaining time
  return 1;
}

/**
 * \brief This is a safety routine to at least grab one goal and score on it
 */
void quickMiddleScore(){
  driveFull.moveVelocity(-100);
  pros::delay(200);
  grabGoal();
  scoreRing();
  dropGoal();
  pros::delay(30000);
 
}

#endif
};  // namespace aon


