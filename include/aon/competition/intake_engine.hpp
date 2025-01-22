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

    void rail_state_machine ()
    {
        // INITIATE
        int state = 0;

        // While Autonomous
        while(true)
        {
            // State 1: Wait for dist_sensor activation
            if (state == 0)
            {
                gate.moveVelocity(0);
                rail.moveVelocity(0);
                
                if (Distance_Sensor.get() < 145){
                    state = 1;
            }    
        }

            
            // State 2: Wait for VS 
            else if (state == 1)
            {   
                gate.moveVelocity(100);
                rail.moveVelocity(100);

                pros::vision_object_s_t obj = Vision_Sensor.get_by_code(0, Red);
                pros::vision_object_s_t obj2 = Vision_Sensor.get_by_code(0, Blue);
                if(obj.signature && obj.width > 1){
                    pros::lcd::set_text(1, "Red detected!");
                    rail.moveVelocity(200);
                    state = 2;
            }
                else if(obj2.signature && obj.width > 1){
                    pros::lcd::set_text(1, "Blue detected!");
                    rail.moveVelocity(100);
                    state = 2;
            }
                else{
                    pros::lcd::set_text(1, "No color detected!");
                    state = 1;
            }
        }
            
            // State 3: Wait before resetting
            else if (state == 2)
            {
                pros::delay(2500);
                state = 0;
            }           
        }
    }

    static void coveyor_init()
    {
        rail_state_machine();
    }
