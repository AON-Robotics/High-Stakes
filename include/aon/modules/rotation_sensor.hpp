#pragma once

#include <math.h>
#include <algorithm>
#include "../constants.hpp"
#include "../globals.hpp"
using namespace std;
//millimeters
#define DIAMETER_OF_ROTATION_WHEEL 0.0457
#define DIAMETER_OF_ROBOT_ROTATION 0.1905
#define PI 3.14159265
#define BACK_ERROR 3462231039250
// int desiredAngle = 49; // Dummy value // from the gps (arctan(delta y / delta x))
// int currentAngle = 23; // Dummy value // from the sensor
// int shortestPath = desiredAngle - currentAngle; // to know if we want to turn left or right
// if(shortestPath < 180 && currentAngle != desiredAngle ){ // Rotate left (i think) // Until we reach the desired angle
//     left_side.move_voltage(12000 * (currentAngle / desiredAngle))
//     right_side.move_voltage(-12000 * (currentAngle / desiredAngle))
// }
// else if(shortestPath >= 180 && currentAngle != desiredAngle){ // Rotate right (i think) // Until we reach the desired angle
//     left_side.move_voltage(-12000 * (currentAngle / desiredAngle))
//     right_side.move_voltage(12000 * (currentAngle / desiredAngle))
// }
/*
void rotating_sequence(int x, int y)
{
    int desired_angle = rotation_degrees(x, y);
    int ;
    
    //Use this formula to find the distance the wheel traveled from the angle given by the sensor
    double r = 4;
    double distance_traveled = 2 * PI * r * (little_angle_traveled/360);
    
    //Use it backwards to find the angle of the big circle (the plane that contains the desired angle)
    // double s = 2 * PI * r (θ/360);
    double angle_traveled = distance_traveled * 360 / (2 * PI * r);

}*/
double vex_rotation_degrees(int x, int y) 
{
    int robot_x = 0;
    int robot_y = 0;

    for (int i = 0; i < 25; i++)
    {
        robot_x += gps.get_status().x;
        robot_y += gps.get_status().y;
    }

    robot_x = robot_x / 25;
    robot_y = robot_y / 25;
    
    // Angle between -180 and 180
    double angle_of_destination = atan2(y - robot_y, x - robot_x) * 180.0/PI;
    // Convert negative angles to those between 180 and 360
    if (angle_of_destination < 0)
    {
        angle_of_destination = 360 + angle_of_destination;            
    }
    // // Inverts cartesian plane
    // angle_of_destination = 360 - angle_of_destination;
    // // Displaces values by 90 degrees
    // angle_of_destination = angle_of_destination + 90.0;
    // // Keeps the values from 0 to 360
    // angle_of_destination = fmod(angle_of_destination, 360);
    // // Double safety
    // if (angle_of_destination >= 360.0)
    // {
    //     angle_of_destination = angle_of_destination - 360.0;
    // }
    angle_of_destination = -1 * angle_of_destination + 90;
    angle_of_destination = fmod(angle_of_destination,360.0);
    if(angle_of_destination < 0) angle_of_destination += 360;
    return angle_of_destination;
}
void rotate_robot(double x, double y)
{
    pros::lcd::print(1, "Starting rotation...");
    pros::lcd::print(2, "Rotate towards (%lf,%lf)", x,y);
    pros::delay(10);
    
    double rotation_angle = vex_rotation_degrees(x, y) * 100;
    pros::lcd::print(3, "Must face at: %lf", rotation_angle);
    pros::delay(5);
    
    encoder_back.set_position(0);
    pros::lcd::print(4, "Encoder position: %lf", (encoder_back.get_position() + BACK_ERROR));
    encoder_back.reset_position();
    pros::delay(5);
    pros::lcd::print(5, "Encoder position: %lf", (encoder_back.get_position() + BACK_ERROR));
    
    double rotated_distance = 0;
    bool go_clockwise;
    double robot_heading = 0;
    // pros::delay(50);

    for (int i = 0; i < 25; i++)
    {
        robot_heading += gps.get_heading();
    }

    robot_heading /= 25.0;
    double arc_length = DIAMETER_OF_ROBOT_ROTATION * PI * (abs(rotation_angle - robot_heading)/360);

    // pros::lcd::print(5, "Robot arc: %lf", arc_length);
    pros::lcd::print(6,"Heading: %lf", robot_heading);
    pros::delay(5);

    // clockwise
    if ((rotation_angle - robot_heading < 0) || (rotation_angle >= robot_heading - 180.0))
    {
        go_clockwise = true;
    }
    else go_clockwise = false;

    pros::delay(50);
    double tolerance = 5;

    while (abs(arc_length - rotated_distance) >= tolerance)
    {
        pros::lcd::print(5, "Encoder position: %lf", (encoder_back.get_position() + BACK_ERROR));
        pros::delay(10);
        if (go_clockwise)
        {
            if (rotated_distance >= abs(arc_length)*0.50)
            {
                left_side.moveVoltage(-2500 * (abs(arc_length - rotated_distance) / arc_length));
                right_side.moveVoltage(2500 * (abs(arc_length - rotated_distance) / arc_length));
            }
            else
            {
                left_side.moveVoltage(2500 * (abs(arc_length - rotated_distance) / arc_length));
                right_side.moveVoltage(-2500 * (abs(arc_length - rotated_distance) / arc_length));
            }
        }
        else
        {
            if (rotated_distance >= abs(arc_length)*0.50)
            {
                left_side.moveVoltage(2500 * (abs(arc_length - rotated_distance) / arc_length));
                right_side.moveVoltage(-2500 * (abs(arc_length - rotated_distance) / arc_length));
            }
            else
            {
                left_side.moveVoltage(-2500 * (abs(arc_length - rotated_distance) / arc_length));
                right_side.moveVoltage(2500 * (abs(arc_length - rotated_distance) / arc_length));
            }
        }

        if(abs(DIAMETER_OF_ROTATION_WHEEL * PI * (encoder_back.get_position()/(100))) >= abs(arc_length))
        {
            left_side.moveVoltage(0);
            right_side.moveVoltage(0);
            pros::delay(1000);
            return;
        }
        
        rotated_distance = DIAMETER_OF_ROTATION_WHEEL * PI * (encoder_back.get_position()/(100));
        //pros::delay(10);
    }
    // left_side.moveVoltage(0);
    // right_side.moveVoltage(0);
    pros::delay(100000);
    pros::lcd::print(7, "Finished rotation");
}
