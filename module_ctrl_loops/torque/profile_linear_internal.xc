/*
 * profile_linear_internal.xc
 *
 *  Created on: Mar 28, 2016
 *      Author: Synapticon
 */

#include <profile_linear_internal.h>
#include <math.h>

int calculate_profile_linear_steps(ProfileLinearParams & profile_linear_params) {
    if (profile_linear_params.v_d >= profile_linear_params.max_value) {
        profile_linear_params.v_d = profile_linear_params.max_value;
    } else if(profile_linear_params.v_d <= -profile_linear_params.max_value) {
        profile_linear_params.v_d = -profile_linear_params.max_value;
    }

    if (profile_linear_params.u >= 0 && profile_linear_params.v_d >= 0) {
        /* both initial and desired velocity - positive case */
        if (profile_linear_params.v_d >= profile_linear_params.u) {
            profile_linear_params.a_d = profile_linear_params.acc;
            profile_linear_params.oldst= 1;
        } else if (profile_linear_params.v_d < profile_linear_params.u) {
            profile_linear_params.a_d = profile_linear_params.dec;
            profile_linear_params.oldst = 2;
        }
    } else if (profile_linear_params.u <= 0 && profile_linear_params.v_d <= 0) {
        /* both initial and desired velocity - negative case */
        if (profile_linear_params.u == 0) {
            profile_linear_params.a_d = profile_linear_params.acc;
            profile_linear_params.oldst = -1;
        } else if(profile_linear_params.v_d == 0) {
            profile_linear_params.a_d = -profile_linear_params.dec;
            profile_linear_params.oldst = -2;
        } else {
            if (profile_linear_params.v_d < profile_linear_params.u) {
                profile_linear_params.a_d = -profile_linear_params.acc;
                profile_linear_params.oldst = -1;
            } else if (profile_linear_params.v_d > profile_linear_params.u) {
                profile_linear_params.a_d = -profile_linear_params.dec;
                profile_linear_params.oldst = -2;
            }
        }
    } else if (profile_linear_params.u > 0 && profile_linear_params.v_d < 0) {
        /* initial and desired velocity - transition from +ve to -ve case */
        if (profile_linear_params.oldst == 2 || profile_linear_params.oldst == -2) {
            profile_linear_params.a_d = profile_linear_params.acc;
            profile_linear_params.oldst = 1;
        } else {
            profile_linear_params.a_d = profile_linear_params.dec;
            profile_linear_params.oldst = 2;
        }
    } else if (profile_linear_params.u < 0 && profile_linear_params.v_d > 0) {
        /* initial and desired velocity - transition from -ve to +ve case */
        if (profile_linear_params.oldst == -1) {
            profile_linear_params.a_d = -profile_linear_params.dec;
            profile_linear_params.oldst =-2;
        } else {
            profile_linear_params.a_d = -profile_linear_params.acc;
            profile_linear_params.oldst = -1;
        }
    }

    profile_linear_params.s_time = .001f;

    // compute time needed
    profile_linear_params.t = (profile_linear_params.v_d - profile_linear_params.u) / profile_linear_params.a_d;

    profile_linear_params.T = profile_linear_params.t / profile_linear_params.s_time;

    if (profile_linear_params.T < 0)
        profile_linear_params.T = -profile_linear_params.T;

    //length = (int) round (T);
    profile_linear_params.s_time = profile_linear_params.t / profile_linear_params.T;

    return (int) round (profile_linear_params.T);
}

int generate_profile_step_torque(ProfileLinearParams profile_linear_params, int step) {
    return (int) round( profile_linear_params.u + profile_linear_params.a_d * profile_linear_params.s_time * step);
}
