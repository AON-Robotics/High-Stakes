#pragma once


#include <cmath>
#include <algorithm>
#include <queue>

#include "../constants.hpp"
#include "../globals.hpp"
#include "../sensing/odometry.hpp"
#include "../controls/pid/pid.hpp"
#include "../controls/holonomic-motion.hpp"
#include "autonomous-routines.hpp"

/*     _______. __   _______   _______    .___________.    ___           _______. __  ___      _______.
    /       ||  | |       \ |   ____|   |           |   /   \         /       ||  |/  /     /       |
   |   (----`|  | |  .--.  ||  |__      `---|  |----`  /  ^  \       |   (----`|  '  /     |   (----`
    \   \    |  | |  |  |  ||   __|         |  |      /  /_\  \       \   \    |    <       \   \    
.----)   |   |  | |  '--'  ||  |____        |  |     /  _____  \  .----)   |   |  .  \  .----)   |   
|_______/    |__| |_______/ |_______|       |__|    /__/     \__\ |_______/    |__|\__\ |_______/    
*/

    void rail_state_machine() {
    // Initialize state
    int state = 0;

    // Autonomous loop
    while (true) {
        if (state == 0) {
            gate.moveVelocity(0);
            rail.moveVelocity(0);

            if (Distance_Sensor.get() < 100) {
                gate.moveVelocity(100);  
                state = 1;
            }
        }

        //Vision Sensor color detection
        else if (state == 1) {
            rail.moveVelocity(100); 
            int object_count = Vision_Sensor.get_object_count();

            if (object_count > 0) {
                pros::vision_object_s_t red_obj = Vision_Sensor.get_by_code(0, Red);
                pros::vision_object_s_t blue_obj = Vision_Sensor.get_by_code(0, Blue);

                if (red_obj.signature == Red) {
                    rail.moveVelocity(200);
                    pros::delay(1000);
                    state = 0;
                }

                else if (blue_obj.signature == Blue) {
                    rail.moveVelocity(100);
                    pros::delay(1000);
                    state = 0;
                } 
            } else {
                state = 1;
            }
        }
    }
}

    static void conveyor_init()
    {
        rail_state_machine();
    }
