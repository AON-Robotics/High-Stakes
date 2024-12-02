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

namespace aon {

#define SAMPLE_SIZE 50

#if USING_15_INCH_ROBOT

double pos_x, pos_y, heading;

class State
{
    private:
        int s_id;
        // double x, y;
        // string action;
        int (*function1)();
        int (*function2)();
        int (*function3)();
        int (*function4)();

    public:
        // Constructor without functions
        State(int s_id/*, double x, double y, string action*/)
        {
            this->s_id = s_id;
            // this->x = x;
            // this->y = y;
            this->function1 = nullptr;
            this->function2 = nullptr;
            this->function3 = nullptr;
            this->function4 = nullptr;
        }

        // Constructor with function pointers
        State(int s_id,/* double x, double y,*/ int (*function1)(), int (*function2)(), int (*function3)(), int (*function4)())
        {
            this->s_id = s_id;
            // this->x = x;
            // this->y = y;
            // this->action = action;  // Uncomment if needed
            this->function1 = function1;
            this->function2 = function2;
            this->function3 = function3;
            this->function4 = function4;
        }

        void runFunctions() const
        {
            function1();
            function2();
            function3();
            function4();
        }
};

// Queue for holding State objects
std::queue<State> state_space;


/*
.______        ______    __    __  .___________. __  .__   __.  _______     _______.
|   _  \      /  __  \  |  |  |  | |           ||  | |  \ |  | |   ____|   /       |
|  |_)  |    |  |  |  | |  |  |  | `---|  |----`|  | |   \|  | |  |__     |   (----`
|      /     |  |  |  | |  |  |  |     |  |     |  | |  . `  | |   __|     \   \    
|  |\  \----.|  `--'  | |  `--'  |     |  |     |  | |  |\   | |  |____.----)   |   
| _| `._____| \______/   \______/      |__|     |__| |__| \__| |_______|_______/    
*/                                                                                    

void primary_routine_init()
{
    // Create the State object directly, no pointers required
    State position_one(0, 
        [](){ return initialReset(); },
        [](){ return move(12); }, 
        [](){ return turn(90); },
        [](){ return move(-12); });

    // Add the state to the state_space queue
    state_space.push(position_one);
}

void primary_routine_runner()
{
    while (!state_space.empty()) {
        // Get the next state
        State current_state = state_space.front();

        // Run the functions in that state
        current_state.runFunctions();

        // Remove the state from the queue
        state_space.pop();
    }
}

int primary_routine_wrapper()
{
    primary_routine_init();
    pros::delay(250);
    primary_routine_runner();
    return 0;
}


/*     _______. __   _______   _______    .___________.    ___           _______. __  ___      _______.
    /       ||  | |       \ |   ____|   |           |   /   \         /       ||  |/  /     /       |
   |   (----`|  | |  .--.  ||  |__      `---|  |----`  /  ^  \       |   (----`|  '  /     |   (----`
    \   \    |  | |  |  |  ||   __|         |  |      /  /_\  \       \   \    |    <       \   \    
.----)   |   |  | |  '--'  ||  |____        |  |     /  _____  \  .----)   |   |  .  \  .----)   |   
|_______/    |__| |_______/ |_______|       |__|    /__/     \__\ |_______/    |__|\__\ |_______/    
*/

int conveyor_belt()
{
    if (true)
    {
        /* code */
    }
    

    return 0;
}


/*
.______       __    __  .__   __. .__   __.  _______ .______      
|   _  \     |  |  |  | |  \ |  | |  \ |  | |   ____||   _  \     
|  |_)  |    |  |  |  | |   \|  | |   \|  | |  |__   |  |_)  |    
|      /     |  |  |  | |  . `  | |  . `  | |   __|  |      /     
|  |\  \----.|  `--'  | |  |\   | |  |\   | |  |____ |  |\  \----.
| _| `._____| \______/  |__| \__| |__| \__| |_______|| _| `._____|
*/

int task_runner()
{
    pros::Task my_routine(primary_routine_wrapper);
    // pros::Task my_conveyor_belt(conveyor_belt);

    return 0;
}

#else
// Other robot size implementations
#endif

}
