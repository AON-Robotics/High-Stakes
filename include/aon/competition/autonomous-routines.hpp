#pragma once

#include <math.h>
#include <algorithm>
#include "../constants.hpp"
#include "../globals.hpp"

#include "../sensing/odometry.hpp"
#include "../controls/pid/pid.hpp"
#include "../controls/holonomic-motion.hpp"

#include <stack>

#define PI 3.14159265
#define POSITIVE_ROTATION_VOLTAGE  2500
#define NEGATIVE_ROTATION_VOLTAGE  -2500

namespace aon {

#if USING_15_INCH_ROBOT

inline void motiontest(){
    {
    auto y_profile_3 = TrapezoidMotionProfile();
    y_profile_3.SetParams(0.20 - gps.get_status().y, MAX_SPEED, 10, 0, 135, 100,
                          0.20);
    auto x_profile_3 = aon::HoldMotionProfile();
    x_profile_3.SetParams(0);
    auto theta_profile_3 = aon::HoldMotionProfile();
    theta_profile_3.SetParams(0);
    aon::holonomic_motion::Move(x_profile_3, y_profile_3, theta_profile_3, 2.0);
    }
        pros::delay(1000);

            {
    auto y_profile_3 = aon::HoldMotionProfile();
    y_profile_3.SetParams(0);
    auto x_profile_3 = TrapezoidMotionProfile();
    x_profile_3.SetParams(0.20 - gps.get_status().x, MAX_SPEED, 10, 0, 135, 100,
                          0.20);
    auto theta_profile_3 = aon::HoldMotionProfile();
    theta_profile_3.SetParams(0);
    aon::holonomic_motion::Move(x_profile_3, y_profile_3, theta_profile_3, 2.0);
        }
}

inline void position(){
    long double pi = 3.14159265358979323846;
    double x = 0.20 - gps.get_status().x;
    double y = 0.20 - gps.get_status().y;
    double startingx = gps.get_status().x;
    double startingy = gps.get_status().y;

      double rev = 1.0;
      double starting_distance = ::std::sqrt(::std::pow(startingx, 2) + ::std::pow(startingy, 2));
      double total_distance = ::std::sqrt(::std::pow(x, 2) + ::std::pow(y, 2));
      double resulting_rev = total_distance / rev; 
      double angle = ::std::asin((::std::sin(90/180.0 * pi) *x) / total_distance) / pi * 180;


    if (x == 0.0 and y == 0.0){
       left_side.moveVoltage(0);
       right_side.moveVoltage(0);
    }
    else{
      //std::cout << " angle: " << angle;
     if(y < 0){
      angle = 180 - angle;
     }
       {
    auto y_profile_4 = aon::HoldMotionProfile();
    y_profile_4.SetParams(0);
    auto x_profile_4 = aon::HoldMotionProfile();
    x_profile_4.SetParams(0);
    auto theta_profile_4 = ExponentialMotionProfile();
    theta_profile_4.SetParams(angle, 3, 0.1);
    aon::holonomic_motion::Move(x_profile_4, y_profile_4, theta_profile_4, 2.0);
        }
    pros::delay(1000);


      
      
      
      /*while(starting_distance < total_distance){
        drive_front_left.moveVoltage(12000);
        drive_back_left.moveVoltage(12000);
        drive_front_right.moveVoltage(12000);
        drive_back_right.moveVoltage(12000);
       }*/

    }

    pros::lcd::print(0, "X Position: %3lf", gps.get_status().x);
    pros::lcd::print(1, "Y Position: %3lf", gps.get_status().y);
    pros::lcd::print(2, "Rotation Degrees: %3lf", gps.get_heading());
    pros::lcd::print(3, "Distance Calculated: %3lf", total_distance);
    pros::lcd::print(4, "Angle Calculated: %3lf", angle);

} 

inline void autonomous_movement_test(){
    pros::lcd::print(0, "X Position: %3lf", gps.get_status().x);
    pros::lcd::print(1, "Y Position: %3lf", gps.get_status().y);
    pros::lcd::print(2, "Rotation Degrees: %3lf", gps.get_heading());

     if(gps.get_status().y > (double)0.35){        
        right_side.moveVoltage(12000);
        left_side.moveVoltage(12000);
        pros::delay(100);
    }
    // else {
    //     drive_front_left.moveVoltage(0);
    //     drive_back_left.moveVoltage(0);
    //     drive_front_right.moveVoltage(0);
    //     drive_back_right.moveVoltage(0);
    // }
}
bool clockwise_rotation = true;

struct Coordinates{
  double x;
  double y;

  Coordinates() {}
  Coordinates(double a, double b) : x(a), y(b) {}
};

  // Returns if the robot reached specified coordinate
  inline bool dest_validation(struct Coordinates destination_pos) {
    if (::std::abs(gps.get_status().x - destination_pos.x) < 0.1 && ::std::abs(gps.get_status().y - destination_pos.y) < 0.1)
    {
      return true;
    }
    return false;
  }

  // Returns true when the robot is oriented correctly
  inline bool angle_validation(double angle_of_destination) {
    if (::std::abs((int)gps.get_heading() - (int)angle_of_destination) <= 1)
    {
      return true;
    }
    return false;
  }

/***********************************************************************************************************************************************************************************/

  //Finds the amount of degrees the robot must rotate to face its objective
  double rotation_degrees(Coordinates destination) 
  {
    // Angle between -180 and 180
    double angle_of_destination = atan2(destination.y - gps.get_status().y, destination.x - gps.get_status().x) * 180.0/PI;

    // Convert negative angles to those between 180 and 360
    if ((int)angle_of_destination < 0)
    {
      angle_of_destination = 360 + angle_of_destination;            
    }

    // Inverts cartesian plane
    angle_of_destination = 360 - angle_of_destination;

    // Displaces values by 90 degrees
    angle_of_destination = angle_of_destination + 90.0;

    // Keeps the values from 0 to 360
    angle_of_destination = fmod(angle_of_destination, 360);

    // Double safety
    if (angle_of_destination >= 360.0)
    {
      angle_of_destination = angle_of_destination - 360.0;
    }

    return angle_of_destination;
  }

  //Finds distance
  double distance(Coordinates destination)
  {
    return ::std::sqrt(
      ::std::pow(gps.get_status().x - destination.x, 2) 
      + ::std::pow(gps.get_status().y - destination.y, 2)
      );
  }
  
  // Rotational movement with directional context
  bool boolean_rotation(double angle)
  {

    pros::lcd::print(1, "Angle: %lf", angle);
    left_side.moveVoltage(0);
    right_side.moveVoltage(0);
    pros::delay(50);

    while (!angle_validation(angle))
    {
      //Dampening
      if (::std::abs((int)gps.get_heading() - (int)angle) <= 3)
      {
        ending_rotation = true;
        
        // Determining the initial angle of the dampening state
        if (!ending_rotation) dampening_angle = ::std::abs(gps.get_heading() - angle);

        // clockwise
        if ((angle - gps.get_heading() > 0) || (angle <= gps.get_heading() - 180.0))
        {
          left_side.moveVoltage(2000 * (int)(::std::abs(gps.get_heading() - angle)/dampening_angle));
          right_side.moveVoltage(-2000 * (int)(::std::abs(gps.get_heading() - angle)/dampening_angle));
        }
        //counterclockwise
        else
        {
          left_side.moveVoltage(-2000 * (int)(::std::abs(gps.get_heading() - angle)/dampening_angle));
          right_side.moveVoltage(2000 * (int)(::std::abs(gps.get_heading() - angle)/dampening_angle));
        }
        break;
      }

      // clockwise
      if ((angle - gps.get_heading() > 0) || (angle <= gps.get_heading() - 180.0))
      {
        left_side.moveVoltage(3250);
        right_side.moveVoltage(-3250);
        ending_rotation = false;
      }
      //counterclockwise
      else
      {
        left_side.moveVoltage(-3250);
        right_side.moveVoltage(3250);
        ending_rotation = false;
      }      
    }

    left_side.moveVoltage(0);
    right_side.moveVoltage(0);
    pros::delay(100);
    return true;
  }

  // Core process (MAKE A COPY OF IT PRIOR MAkING A SPECIFIC ROUTINE!)
  inline void new_traverse()
  {
    gps.set_data_rate(15);
    pros::delay(2000);

    // Locations
    ::std::stack<Coordinates> route;
    Coordinates destination(0.9, 0);
    route.push(destination);

    double angle_of_destination;

    while (!route.empty())
    {
        // Routine
        while (::std::abs(distance(route.top())) >= 0.1)
        {
          // Calculates angle and rotates 
          if (boolean_rotation(rotation_degrees(route.top())))
          {
            // left_side.moveVoltage(7000);
            // right_side.moveVoltage(7000);
          }
          
        }

      left_side.moveVoltage(0);
      right_side.moveVoltage(0);
      route.pop();
      pros::delay(100);
    }
  }

  inline void run_forward()
  {
    left_side.moveVoltage(5000);
    right_side.moveVoltage(5000);
    pros::delay(4000);
    left_side.moveVoltage(0);
    right_side.moveVoltage(0);
  }
  
  inline void blockade_routine() ////////////////////////////////////
  {
    gps.set_offset(0.11, 0.127);
    gps.set_data_rate(15);
    pros::delay(2000);

    // Locations
    ::std::stack<Coordinates> route;
    if(::std::abs(gps.get_heading() - 270) <= 2)
    {
      Coordinates destination(-0.9,0.9);
      route.push(destination);

      destination.x = -1.2;
      destination.y = 1.2;
      route.push(destination);

      destination.x = -0.9;
      destination.y = 1.5;
      route.push(destination);

      destination.x = 0;
      destination.y = 1.5;
      route.push(destination);

    }
    else if (::std::abs(gps.get_heading() - 90) <= 2)
    {
      Coordinates destination(0.9,-0.9);
      route.push(destination);

      destination.x = 1.2;
      destination.y = -1.2;
      route.push(destination);

      destination.x = 0.9;
      destination.y = -1.5;
      route.push(destination);

      destination.x = 0;
      destination.y = -1.5;
      route.push(destination);
    }

    double angle_of_destination;

    while (!route.empty())
    {
        if(route.size() == 4)
        {
          left_side.moveVoltage(1000);
          right_side.moveVoltage(1000);
          pros::delay(500);
          left_side.moveVoltage(0);
          right_side.moveVoltage(0);
        }

        // Routine
        while (::std::abs(distance(route.top())) >= 0.1)
        {
          // Calculates angle and rotates 
          if (boolean_rotation(rotation_degrees(route.top())))
          {
            left_side.moveVoltage(7000);
            right_side.moveVoltage(7000);
          }
          
        }

      left_side.moveVoltage(0);
      right_side.moveVoltage(0);
      route.pop();

      if(route.empty())
      {
        extended_right = true;
        extended_left = true;
        right_hand.set_value(1);
        left_hand.set_value(1);
      }

      pros::delay(100);
    }
  }

inline void programming_skills() {

}
#else

  /*
  * General idea:
  *
  * 1) Call a custom function that verifies if the robot is in the target destination.
  * 2) If not, calculate the distance and rotation degrees.
  * 3) Finally, rotate (if necessary) and move the robot until it reaches the destination.
  */

  /*
  * Regarding the GPS:
  *
  * According to the documentation, the positions or coordinates inside the field
  * are measured in m (from -1.8m to 1.8m) with respect to the origin. THE Y-AXIS EXTENDS FROM 0 DEGREES
  * TO 180 (WALLS 4 AND 2). The yaw is what will contain the angle of rotation since
  * it measures the rotation around the vertical axis. 
  */

 
  bool clockwise_rotation = true;

  struct Coordinates{
  double x;
  double y;

  Coordinates() {}
  Coordinates(double a, double b) : x(a), y(b) {}
};

  // Returns if the robot reached specified coordinate
  inline bool dest_validation(struct Coordinates destination_pos) {
    if (::std::abs(gps.get_status().x - destination_pos.x) < 0.1 && ::std::abs(gps.get_status().y - destination_pos.y) < 0.1)
    {
      return true;
    }
    return false;
  }

  // Returns true when the robot is oriented correctly
  inline bool angle_validation(double angle_of_destination) {
    if (::std::abs((int)gps.get_heading() - (int)angle_of_destination) <= 1)
    {
      return true;
    }
    return false;
  }

  /*
  * NOTES: This is to validate if the robot is oriented correctly.
  */
  inline void axis_rotation(double angle_of_destination){

    pros::lcd::print(0, "The angle of the robot is: %3lf", gps.get_heading());
    pros::lcd::print(1, "The destination's angle is: %3lf", angle_of_destination);
    pros::lcd::print(2, "Robot: x = %3lf, y = %3lf", gps.get_status().x, gps.get_status().y);
    pros::delay(10);

    while(!angle_validation(angle_of_destination)){
        //Right
        if(clockwise_rotation)
        {
          left_side.moveVoltage(NEGATIVE_ROTATION_VOLTAGE);
          right_side.moveVoltage(POSITIVE_ROTATION_VOLTAGE);
        }
        //Left
        else
        {
          left_side.moveVoltage(POSITIVE_ROTATION_VOLTAGE);
          right_side.moveVoltage(NEGATIVE_ROTATION_VOLTAGE);
        }
      }

    left_side.moveVoltage(0);
    right_side.moveVoltage(0);
  }

  /*
  * NOTES: 
  */
  inline void traverse() {
    pros::delay(2000);

    ::std::stack<Coordinates> locations;

    Coordinates destination (0.35, 0.0);
    locations.push(destination);
    // destination.x = 0.35;
    // destination.y = 0.0;

    Coordinates temp;
    
    // Initialized variables that will be used to find the rotation angle
    Coordinates adjusted_destination;
    double angle_of_destination;
    
    gps.set_offset(0, 0);

    while (1/*!dest_validation(destination) && locations.size() != 0*/)
    {
      /*
      * In the following we find the differences between the robot's position and that of the destination.
      * It basically sets the robot to be the center of the cartesian plane.
      */
      
      temp = Coordinates(locations.top().x, locations.top().y);
      locations.pop();
      
      adjusted_destination.x = destination.x - gps.get_status().x;
      adjusted_destination.y = destination.y - gps.get_status().y;

      pros::delay(1);

      /*
      * With atan(y/x) we should be able to find at which angle the destination is located from the robot
      * NOTE: atan returns a radian, the 180/PI converts it into degrees.
      */
      angle_of_destination = ::std::atan2(adjusted_destination.y, adjusted_destination.x) * 180.0/PI;
      clockwise_rotation = angle_of_destination >= 0;

      pros::delay(1);

      /*
      * atan2() returns a value between -180 to 180. Therefore, we have to add 360 degrees when it is negative
      * to find/translate the angle into those of 180 to 360.
      */    
      if((adjusted_destination.x < 0 && adjusted_destination.y < 0) || (adjusted_destination.x > 0 && adjusted_destination.y < 0))
      {
        angle_of_destination += 360.0;
      }

      /*
      * Basically it converts it to its clockwise counterpart and displaces the angle by 90 degrees.
      * Just look at the cartesian plane from the gps documentation and play with the formula on
      * desmos to understand.
      */
      angle_of_destination = fmod(-angle_of_destination + 90.0, 360.0);

      axis_rotation(angle_of_destination);

    }
    
  }


/***********************************************************************************************************************************************************************************/

  //Finds the amount of degrees the robot must rotate to face its objective
  double rotation_degrees(Coordinates destination) 
  {
    // Angle between -180 and 180
    double angle_of_destination = atan2(destination.y - gps.get_status().y, destination.x - gps.get_status().x) * 180.0/PI;

    // Convert negative angles to those between 180 and 360
    if ((int)angle_of_destination < 0)
    {
      angle_of_destination = 360 + angle_of_destination;            
    }

    // Inverts cartesian plane
    angle_of_destination = 360 - angle_of_destination;

    // Displaces values by 90 degrees
    angle_of_destination = angle_of_destination + 90.0;

    // Keeps the values from 0 to 360
    angle_of_destination = fmod(angle_of_destination, 360);

    // Double safety
    if (angle_of_destination >= 360.0)
    {
      angle_of_destination = angle_of_destination - 360.0;
    }

    return angle_of_destination;
  }

  //Finds distance
  double distance(Coordinates destination)
  {
    return ::std::sqrt(
      ::std::pow(gps.get_status().x - destination.x, 2) 
      + ::std::pow(gps.get_status().y - destination.y, 2)
      );
  }
  
  // Rotational movement with directional context
  bool boolean_rotation(double angle)
  {

    pros::lcd::print(1, "Angle: %lf", angle);
    left_side.moveVoltage(0);
    right_side.moveVoltage(0);
    pros::delay(50);

    while (!angle_validation(angle))
    {
      //Dampening
      if (::std::abs((int)gps.get_heading() - (int)angle) <= 3)
      {
        ending_rotation = true;
        
        // Determining the initial angle of the dampening state
        if (!ending_rotation) dampening_angle = ::std::abs(gps.get_heading() - angle);

        // clockwise
        if ((angle - gps.get_heading() > 0) || (angle <= gps.get_heading() - 180.0))
        {
          left_side.moveVoltage(2000 * (int)(::std::abs(gps.get_heading() - angle)/dampening_angle));
          right_side.moveVoltage(-2000 * (int)(::std::abs(gps.get_heading() - angle)/dampening_angle));
        }
        //counterclockwise
        else
        {
          left_side.moveVoltage(-2000 * (int)(::std::abs(gps.get_heading() - angle)/dampening_angle));
          right_side.moveVoltage(2000 * (int)(::std::abs(gps.get_heading() - angle)/dampening_angle));
        }
        break;
      }

      // clockwise
      if ((angle - gps.get_heading() > 0) || (angle <= gps.get_heading() - 180.0))
      {
        left_side.moveVoltage(3250);
        right_side.moveVoltage(-3250);
        ending_rotation = false;
      }
      //counterclockwise
      else
      {
        left_side.moveVoltage(-3250);
        right_side.moveVoltage(3250);
        ending_rotation = false;
      }      
    }

    left_side.moveVoltage(0);
    right_side.moveVoltage(0);
    return true;
  }

  // Core process (MAKE A COPY OF IT PRIOR MAING A SPECIFIC ROUTINE!)
  inline void new_traverse()
  {
    gps.set_data_rate(15);
    pros::delay(2000);

    // Locations
    ::std::stack<Coordinates> route;
    Coordinates destination(0.9, 0);
    route.push(destination);

    double angle_of_destination;

    while (!route.empty())
    {
        // Routine
        while (::std::abs(distance(route.top())) >= 0.1)
        {
          // Calculates angle and rotates 
          if (boolean_rotation(rotation_degrees(route.top())))
          {
            // left_side.moveVoltage(7000);
            // right_side.moveVoltage(7000);
          }
          
        }

      left_side.moveVoltage(0);
      right_side.moveVoltage(0);
      route.pop();
      pros::delay(100);
    }
    
  }


#endif
};  // namespace aon