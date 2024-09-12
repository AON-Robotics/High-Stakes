#pragma once

#include <math.h>
#include <algorithm>
#include "../constants.hpp"
#include "../globals.hpp"

#define DIAMETER_OF_WHEELS 0.04572
#define PI 3.14159265


using namespace std;

void move_forward(double x, double y)
{

    pros::lcd::print(1, "Moving forward...");
    
    encoder_left.set_position(0);
    encoder_right.set_position(0);

    int revolutions = 0;
    double robot_x = 0;
    double robot_y = 0;

    pros::delay(50);

    for(int i = 0; i < 25; i++)
    {
        robot_x+=gps.get_status().x;
        robot_y+=gps.get_status().y;
    }

    robot_x/=25;
    robot_y/=25;

    double distance = sqrt(pow(x-robot_x, 2)+pow(y-robot_y, 2));
    double traveled = 0;

    while (traveled <= distance)
    {
        left_side.moveVelocity(500 * (abs(distance - traveled) / distance));
        right_side.moveVelocity(500 * (abs(distance - traveled) / distance));

        traveled = DIAMETER_OF_WHEELS * PI * (encoder_left.get_position() + encoder_right.get_position()) / (2 * 360 * 100);
    }
    left_side.moveVelocity(0);
    right_side.moveVelocity(0);

    pros::lcd::print(1, "Reached coordinate");
}