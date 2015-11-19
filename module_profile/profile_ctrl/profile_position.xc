/**
 * @file profile_position.xc
 * @brief Profile Position Control functions
 *      Implements position profile control function
 * @author Synapticon GmbH <support@synapticon.com>
*/

#include <refclk.h>
#include <xscope.h>
#include <internal_config.h>
#include <statemachine.h>
#include <drive_modes.h>
#include <print.h>
#include <profile.h>
#include <profile_control.h>

void set_profile_position(int target_position, int velocity, int acceleration, int deceleration,
                          int sensor_select,  interface PositionControlInterface client i_position_control )
{
    int i;
    timer t;
    unsigned int time;
    int steps;
    int position_ramp;

    int actual_position = 0;

    int init_state = i_position_control.check_busy();//__check_position_init(c_position_ctrl);


    if (init_state == INIT_BUSY)
    {
        i_position_control.set_position_sensor(sensor_select);
        init_state = init_position_control(i_position_control);
    }

    if(init_state == INIT)
    {
        actual_position = i_position_control.get_position();
        steps = init_position_profile(target_position, actual_position, velocity, acceleration, deceleration);
        t :> time;
        for(i = 1; i < steps; i++)
        {
            position_ramp = position_profile_generate(i);
            i_position_control.set_position(position_ramp);
            actual_position = i_position_control.get_position();
            t when timerafter(time + MSEC_STD) :> time;
            /*xscope_int(0, actual_position);
              xscope_int(1, position_ramp);*/
        }
        t when timerafter(time + 30 * MSEC_STD) :> time;
    }
}
