/**
 * @file profile_position_quick-stop.c
 * @brief Quick stop Profile Generation for Position
 * @author Pavan Kanajar <pkanajar@synapticon.com>
*/

#include <profile.h>

struct {
    float qi;
    float qf;         // user input variables

    float ts;
    float ci;         // motion profile constants

    float tb, t_int;  // motion profile params

    float q;
    float samp;

    float cur_pos_s;
    float acc;
} pos_param_s;      // position quick stop parameters



int init_quick_stop_position_profile(int actual_velocity, int actual_position, int max_deceleration)  //emergency stop
{
    pos_param_s.qi = 0;
    pos_param_s.qf = (float) actual_velocity;   //always positive -ticks/s

    if(pos_param_s.qf < 0) {
        pos_param_s.qf = 0 - pos_param_s.qf;
    }

    pos_param_s.acc = pos_param_s.qf * 10;                  //ticks m/s
    if(pos_param_s.acc > max_deceleration) {
        pos_param_s.acc = max_deceleration;
    }

    pos_param_s.cur_pos_s = (float) actual_position; //ticks

    pos_param_s.tb = pos_param_s.qf / pos_param_s.acc;

    pos_param_s.ci = - pos_param_s.acc / 2;
    pos_param_s.samp = pos_param_s.tb/1.0e-3;
    pos_param_s.t_int = pos_param_s.tb/pos_param_s.samp;
    if(pos_param_s.samp < 0) {
        pos_param_s.samp = 0 - pos_param_s.samp;
    }
    return (int)  round((pos_param_s.samp));
}

int quick_stop_position_profile_generate(int steps, int actual_velocity)
{
    pos_param_s.ts = pos_param_s.t_int * steps;
    pos_param_s.q = pos_param_s.qf * pos_param_s.ts + pos_param_s.ci * pos_param_s.ts * pos_param_s.ts;
    if(actual_velocity >= 0) {
        return (int) round( pos_param_s.cur_pos_s + pos_param_s.q);
    } else if(actual_velocity < 0) {
        return (int) round( pos_param_s.cur_pos_s - pos_param_s.q);
    }
    return 0;
}

