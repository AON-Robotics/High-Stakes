#pragma once

#include <math.h>
#include <algorithm>
#include "../constants.hpp"
#include "../globals.hpp"

#include "../sensing/odometry.hpp"
#include "../controls/pid/pid.hpp"
#include "../controls/holonomic-motion.hpp"

#include "../modules/gps.hpp"
#include "../modules/rotation_sensor.hpp"
#include "../modules/traverse.hpp"

using namespace std;
#include <queue>
/*

Components integrated into the routine

- Motors (encoder will be actively used)
- Rotation Sensor
- GPS Sensor

- Distance Sensor
- Limit Switch

*/

// Globals

namespace aon::modular_autonomous
{
    // Version 2
    class Next_State
    {
        private:
        int s_id;
        double x, y;
        // tring action;
        function<void(double, double)> function1;
        function<void(double, double)> function2;
        function<void(double, double)> function3;

        public:
        /*
        Next_State(int s_id, double x, double y, string action)
        {
            this->s_id = s_id;
            this->x = x;
            this->y = y;
            this->action = action;
            this->function1 = NULL;
            this->function2 = NULL;
            this->function3 = NULL;
        }
        */
        // To be discussed...
        Next_State(int s_id, double x, double y, function<void(int, int)> function1, function<void(int, int)> function2, function<void(int, int)> function3)
        {
            this->s_id = s_id;
            this->x = x;
            this->y = y;
            // this->action = "";
            // Functions can be added if they are need for a certain space
            this->function1 = function1;
            this->function2 = function2;
            this->function3 = function3;
        }

        int get_s_id()
        {
            return s_id;
        }

        double get_x()
        {
            return x;
        }

        double get_y()
        {
            return y;
        }

        // string get_action()
        // {
        //     return action;
        // }

        void execute()
        {
            if(function1 != NULL)
            {
                // Runs the stored function
                function1(x, y);
            } 
            if(function2 != NULL)
            {
                // Runs the stored function
                function2(x, y);
            }
            if(function3 != NULL)
            {
                // Runs the stored function
                function3(x, y);
            }
        }
    };

    // Version 1
    // struct state
    // {
    //     int s_id;
    //     double x, y;
    //     string action;

    //     state() {}
    //     state(int a,double b, double c, string d) : s_id(a), x(b), y(c), action(d) {}
    // };
    

    // // Manages the execution of the task at hand
    // void action_handler(string action)
    // {
    //     if (action.compare("traverse"))
    //     {
    //         /* code */
    //     }
    //     else if (action.compare("rotate"))
    //     {
    //         /* code */
    //     }
    //     else if (action.compare("wait"))
    //     {
    //         /* code */
    //     }
    //     else if (action.compare("score"))
    //     {
    //         /* code */
    //     }
        
    // }

    queue<Next_State> state_space;
    


    // Runs autonomous
    // void runner(void *param)
    void runner()
    {

        rotate_robot(0,0);
        // function<void(double,double)> func1 = rotate_robot;
        // function<void(double,double)> func2 = move_forward;
        // state_space.push(Next_State(1,0,0,func1,func2,NULL));

        // while (!state_space.empty())
        // {
        //     state_space.front().execute();
        //     state_space.pop();
        // }
        
    }

    // Responsible of collecting triball if possible
    void collect(void *param)
    {
        
    }

    // Initiates pararell tasks (call in main)
    void start_autonomous()
    {
        // pros::Task my_Task(collect, NULL, "catch_triball");
        // pros::Task my_Task(runner, NULL, "autonomous");

        // encoder_back.reset_position();
        // while (encoder_back.get_position() < 90 * 100)
        // {
        //     left_side.moveVoltage(12000);
        //     right_side.moveVoltage(-12000);
        // }
        // left_side.moveVoltage(0);
        // right_side.moveVoltage(0);

        // while(1)

        
            // int x2, y2 = 0;
            // for(int i = 0; i < 25; i++){
            //     x2 += gps.get_status().x;
            //     y2 += gps.get_status().y;
            // }
            // x2 = x2 / 25;
            // y2 = y2 / 25;

            // double distances = sqrt(pow((0-x2),2) + pow((0-y2),2));// Distance measured with distance formula
            // double actual_distance = 0.0; // Distance travelled in meters
            // double distance = .25;

            
            //     encoder_left.reset();
            //     encoder_right.reset();
            //     while(actual_distance <= distance){
            //         actual_distance = (((encoder_left.get_position() + encoder_right.get_position()) / 2) / 36000) * 0.04826;
            //         left_side.moveVoltage((distance-actual_distance)/distance * 2000);
            //         right_side.moveVoltage((distance-actual_distance)/distance * 2000);                                   
                            
            
        // }

        move_forward(0,0);

        // runner();
    }

}