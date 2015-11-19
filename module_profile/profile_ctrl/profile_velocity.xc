/**
 * @file profile_velocity.xc
 * @brief Profile Velocity Control functions
 *      Implements velocity profile control function
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
//#define debug_print


void set_profile_velocity(int target_velocity, int acceleration, int deceleration, int max_profile_velocity, interface VelocityControlInterface client i_velocity_control)
{
    int actual_velocity;
    timer t;
    unsigned int time;
    int steps = 0;
    int velocity_ramp;
    int i;
    int init_state = i_velocity_control.check_busy(); //__check_velocity_init(c_velocity_ctrl);
    while(init_state == INIT_BUSY)
    {
        init_state = init_velocity_control(i_velocity_control);
        /*      if(init_state == INIT)
                printstrln("velocity control intialized");
                else
                printstrln("intialize velocity control failed");*/
    }


    if(init_state == INIT)
    {
        actual_velocity = i_velocity_control.get_velocity();
        steps = init_velocity_profile(target_velocity, actual_velocity, acceleration, deceleration, max_profile_velocity);
        t :> time;
        for(i = 1; i < steps; i++) {
            velocity_ramp = velocity_profile_generate(i);
            i_velocity_control.set_velocity(velocity_ramp);
            actual_velocity = i_velocity_control.get_velocity();

            t when timerafter(time + MSEC_STD) :> time;

            /*xscope_int(0, actual_velocity);
              xscope_int(1, velocity_ramp);*/
        }
	if (target_velocity == 0) {
	    i_velocity_control.set_velocity(target_velocity);
        }
        t when timerafter(time + 30 * MSEC_STD) :> time;
    }
}
