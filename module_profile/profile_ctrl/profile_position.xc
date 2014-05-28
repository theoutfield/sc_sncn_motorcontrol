/**
 * \file profile_position.xc
 * \brief Profile Position Control functions
 *      Implements position profile control function
 * \author Pavan Kanajar <pkanajar@synapticon.com>
*/

#include "position_ctrl_client.h"
#include "refclk.h"
#include <xscope_wrapper.h>
#include <internal_config.h>
#include <drive_config.h>
#include "print.h"
#include <profile.h>
#include <profile_control.h>

void set_profile_position(int target_position, int velocity, int acceleration, int deceleration,
                          int sensor_select, chanend c_position_ctrl)
{
    int i;
    timer t;
    unsigned int time;
    int steps;
    int position_ramp;

    int actual_position = 0;

    int init_state = __check_position_init(c_position_ctrl);

    while(init_state == INIT_BUSY)
    {
        set_position_sensor(sensor_select, c_position_ctrl);
        init_state = init_position_control(c_position_ctrl);
        /*if(init_state == INIT)
          printstrln("position control intialized");
          else
          printstrln("intialize position control failed");*/
    }

    if(init_state == INIT)
    {
        actual_position = get_position(c_position_ctrl);
        steps = init_position_profile(target_position, actual_position, velocity, acceleration, deceleration);
        t :> time;
        for(i = 1; i < steps; i++)
        {
            position_ramp = position_profile_generate(i);
            set_position(position_ramp, c_position_ctrl);
            actual_position = get_position(c_position_ctrl);
            t when timerafter(time + MSEC_STD) :> time;
            /*xscope_int(0, actual_position);
              xscope_int(1, position_ramp);*/
        }
        t when timerafter(time + 30 * MSEC_STD) :> time;
    }
}
