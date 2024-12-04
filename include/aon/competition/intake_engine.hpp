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
                
                if (dist_sensor.get_value())
                {
                    state = 1;
                }
                
            }


            
            // State 2: Wait for limit_switch activation
            else if (state == 1)
            {   
                gate.moveVelocity(100);
                rail.moveVelocity(100);
                if(limit_switch.get_value()){
                    
                    state = 2;

                }
            }

            // State 3: VS verdict
            else if (state == 2)
            {
                gate.moveVelocity(0);
                /*
                    If vision sensor detects incorrect color
                    intake.moveVelocity(200);
                */
                state = 3;
            }
            

            // State 4: Wait before resetting
            else if (state == 3)
            {
                pros::delay(2500);
                state = 0;
            }
            

        }
    }

    static void coveyor_init ()
    {
        rail_state_machine();
    }
