/*
 * profile_velocity_internal.xc
 *
 *  Created on: Mar 24, 2016
 *      Author: Synapticon
 */

#include <profile_velocity_internal.h>
#include <math.h>

int calculate_profile_steps(ProfileVelocityParams & profile_velocity_params) {
    if (profile_velocity_params.v_d >= profile_velocity_params.max_velocity) {
        profile_velocity_params.v_d = profile_velocity_params.max_velocity;
    } else if (profile_velocity_params.v_d <= -profile_velocity_params.max_velocity) {
        profile_velocity_params.v_d = 0-profile_velocity_params.max_velocity;
    }
    /* both initial and desired velocity - positive case */
    if (profile_velocity_params.u>=0 && profile_velocity_params.v_d >=0) {
        if(profile_velocity_params.v_d >= profile_velocity_params.u) {
            profile_velocity_params.a_d = profile_velocity_params.acc;
            profile_velocity_params.oldst= 1;
        } else if(profile_velocity_params.v_d < profile_velocity_params.u) {
            profile_velocity_params.a_d = profile_velocity_params.dec;
            profile_velocity_params.oldst= 2;
        }
    } else if(profile_velocity_params.u<=0 && profile_velocity_params.v_d <=0) {
        /* both initial and desired velocity - negative case */
        if (profile_velocity_params.u==0) {
            profile_velocity_params.a_d = profile_velocity_params.acc;
            profile_velocity_params.oldst= -1;
        } else if (profile_velocity_params.v_d==0) {
            profile_velocity_params.a_d = -profile_velocity_params.dec;
            profile_velocity_params.oldst= -2;
        } else {
            if (profile_velocity_params.v_d < profile_velocity_params.u) {
                profile_velocity_params.a_d = -profile_velocity_params.acc;
                profile_velocity_params.oldst= -1;
            } else if (profile_velocity_params.v_d > profile_velocity_params.u) {
                profile_velocity_params.a_d = -profile_velocity_params.dec;
                profile_velocity_params.oldst= -2;
            }
        }
    } else if(profile_velocity_params.u>0 && profile_velocity_params.v_d<0) {
        /* initial and desired velocity - transition from +ve to -ve case */
        if (profile_velocity_params.oldst==2 || profile_velocity_params.oldst==-2) {
            profile_velocity_params.a_d = profile_velocity_params.acc;
            profile_velocity_params.oldst= 1;
        } else {
            profile_velocity_params.a_d = profile_velocity_params.dec;
            profile_velocity_params.oldst= 2;
        }
    } else if(profile_velocity_params.u<0 && profile_velocity_params.v_d>0) {
        /* initial and desired velocity - transition from -ve to +ve case */
        if(profile_velocity_params.oldst==-1) {
            profile_velocity_params.a_d = -profile_velocity_params.dec;
            profile_velocity_params.oldst=-2;
        } else {
            profile_velocity_params.a_d = -profile_velocity_params.acc;
            profile_velocity_params.oldst= -1;
        }
    }

    profile_velocity_params.s_time = .001f;

    // compute time needed
    profile_velocity_params.t = (profile_velocity_params.v_d - profile_velocity_params.u)/profile_velocity_params.a_d;

    profile_velocity_params.T = profile_velocity_params.t/profile_velocity_params.s_time;

    if (profile_velocity_params.T<0)
        profile_velocity_params.T = -profile_velocity_params.T;

    //length = (int) round (T);
    profile_velocity_params.s_time = profile_velocity_params.t/profile_velocity_params.T;

    return (int) round (profile_velocity_params.T);
}

int generate_profile_step_velocity(ProfileVelocityParams profile_velocity_params, int step) {
    return (int) round( profile_velocity_params.u + profile_velocity_params.a_d * profile_velocity_params.s_time * step);
}
