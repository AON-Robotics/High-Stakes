#pragma once

#include <math.h>
#include <algorithm>
#include "../constants.hpp"
#include "../globals.hpp"

#define PI 3.14159265

struct Positioning{
  double x;
  double y;
  double heading;

  Positioning() {}
  Positioning(double a, double b, double c) : x(a), y(b), heading(c) {}
};


struct Positioning extract_data()
{
    int x, y, heading = 0;
    int sample_ammount = 25;

    //Gets average
    for(int i = 0; i < sample_ammount; i++)
    {
        x += gps.get_status().x;
        y += gps.get_status().y;
        heading += gps.get_heading();
    }

    x = x/sample_ammount;
    y = y/sample_ammount;
    heading = heading/sample_ammount;

    return Positioning(x/sample_ammount, y/sample_ammount, heading/sample_ammount);
}

/*
 _     _____ ____    _    ______   __   ____ ___  ____  _____ 
| |   | ____/ ___|  / \  / ___\ \ / /  / ___/ _ \|  _ \| ____|
| |   |  _|| |  _  / _ \| |    \ V /  | |  | | | | | | |  _|  
| |___| |__| |_| |/ ___ \ |___  | |   | |__| |_| | |_| | |___ 
|_____|_____\____/_/   \_\____| |_|    \____\___/|____/|_____|
*/

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