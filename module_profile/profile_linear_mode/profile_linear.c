/**
 * @file profile_linear.c
 * @brief Profile Generation
 *      Implements Torque profile based on linear functions.
 * @author Pavan Kanajar <pkanajar@synapticon.com>
*/

#include <profile.h>
#include <stdio.h>


struct
{
    float max_acceleration, max_deceleration;   // max allowed acceleration & deceleration
    float acc, dec;                             // acceleration & deceleration input
    float u;                                    // initial value
    float v_d;                                  // desired value
    float a_d;                                  // desired acceleration
    float t;                                    // time
    float T;                                    // total no. of Samples
    float s_time;                               // sampling time
    int oldst;                                  // old state of acc/dec
} profile_linear_params;

int init_linear_profile(int target_value, int actual_value, int acceleration, int deceleration, int max_value)
{
    profile_linear_params.u = (float) actual_value;
    profile_linear_params.acc = (float) acceleration;
    profile_linear_params.dec = (float) deceleration;

    if (target_value >= max_value) {
        target_value = max_value;
    } else if(target_value <= -max_value) {
        target_value = 0-max_value;
    }

    profile_linear_params.v_d = (float) target_value;
    //printf("srta");
    //printf("\n%d\n",target_value);

    if(profile_linear_params.u>=0 && profile_linear_params.v_d >=0) {
        /* both initial and desired velocity - positive case */
        if (profile_linear_params.v_d >= profile_linear_params.u) {
            profile_linear_params.a_d = profile_linear_params.acc;
            profile_linear_params.oldst= 1;
        } else if (profile_linear_params.v_d < profile_linear_params.u) {
            profile_linear_params.a_d = profile_linear_params.dec;
            profile_linear_params.oldst= 2;
        }
    } else if(profile_linear_params.u<=0 && profile_linear_params.v_d <=0) {
        /* both initial and desired velocity - negative case */
        if (profile_linear_params.u==0) {
            profile_linear_params.a_d = profile_linear_params.acc;
            profile_linear_params.oldst= -1;
        } else if(profile_linear_params.v_d==0) {
            profile_linear_params.a_d = -profile_linear_params.dec;
            profile_linear_params.oldst= -2;
        } else {
            if (profile_linear_params.v_d < profile_linear_params.u) {
                profile_linear_params.a_d = -profile_linear_params.acc;
                profile_linear_params.oldst= -1;
            } else if (profile_linear_params.v_d > profile_linear_params.u) {
                profile_linear_params.a_d = -profile_linear_params.dec;
                profile_linear_params.oldst= -2;
            }
        }
    } else if (profile_linear_params.u>0 && profile_linear_params.v_d<0) {
        /* initial and desired velocity - transition from +ve to -ve case */
        if (profile_linear_params.oldst==2 || profile_linear_params.oldst==-2) {
            profile_linear_params.a_d = profile_linear_params.acc;
            profile_linear_params.oldst= 1;
        } else {
            profile_linear_params.a_d = profile_linear_params.dec;
            profile_linear_params.oldst= 2;
        }
    } else if(profile_linear_params.u<0 && profile_linear_params.v_d>0) {
        /* initial and desired velocity - transition from -ve to +ve case */
        if(profile_linear_params.oldst==-1) {
            profile_linear_params.a_d = -profile_linear_params.dec;
            profile_linear_params.oldst=-2;
        } else {
            profile_linear_params.a_d = -profile_linear_params.acc;
            profile_linear_params.oldst= -1;
        }
    }

    profile_linear_params.s_time = .001f;

    // compute time needed
    profile_linear_params.t = (profile_linear_params.v_d - profile_linear_params.u)/profile_linear_params.a_d;

    profile_linear_params.T = profile_linear_params.t/profile_linear_params.s_time;

    if (profile_linear_params.T<0)
        profile_linear_params.T = -profile_linear_params.T;

    //length = (int) round (T);
    profile_linear_params.s_time = profile_linear_params.t/profile_linear_params.T;

    return (int) round (profile_linear_params.T);
}

int linear_profile_generate(int step)
{
    return (int) round( profile_linear_params.u + profile_linear_params.a_d * profile_linear_params.s_time * step);
}

int __init_linear_profile_float(float target_value, float actual_value, float acceleration,
                                float deceleration, float max_value, profile_linear_param *profile_linear_params)
{
    profile_linear_params->u =  actual_value;
    // profile_linear_params->v_d = (float) target_value;
    profile_linear_params->acc = acceleration;
    profile_linear_params->dec = deceleration;

    if(target_value >= max_value)
        target_value = max_value;
    else if(target_value <= -max_value)
        target_value = 0-max_value;
    profile_linear_params->v_d = target_value;
    //printf("srta");
    //printf("\n%d\n",target_value);

    if(profile_linear_params->u>=0 && profile_linear_params->v_d >=0) {
        /* both initial and desired velocity - positive case */
        if (profile_linear_params->v_d >= profile_linear_params->u) {
            profile_linear_params->a_d = profile_linear_params->acc;
            profile_linear_params->oldst= 1;
        } else if (profile_linear_params->v_d < profile_linear_params->u) {
            profile_linear_params->a_d = profile_linear_params->dec;
            profile_linear_params->oldst= 2;
        }
    }

    else if (profile_linear_params->u<=0 && profile_linear_params->v_d <=0) {
        /* both initial and desired velocity - negative case */
        if (profile_linear_params->u==0) {
            profile_linear_params->a_d = profile_linear_params->acc;
            profile_linear_params->oldst= -1;
        } else if (profile_linear_params->v_d==0) {
            profile_linear_params->a_d = -profile_linear_params->dec;
            profile_linear_params->oldst= -2;
        } else {
            if (profile_linear_params->v_d < profile_linear_params->u) {
                profile_linear_params->a_d = -profile_linear_params->acc;
                profile_linear_params->oldst= -1;
            } else if(profile_linear_params->v_d > profile_linear_params->u) {
                profile_linear_params->a_d = -profile_linear_params->dec;
                profile_linear_params->oldst= -2;
            }
        }
    } else if(profile_linear_params->u>0 && profile_linear_params->v_d<0) {
        /* initial and desired velocity - transition from +ve to -ve case */
        if(profile_linear_params->oldst==2 || profile_linear_params->oldst==-2) {
            profile_linear_params->a_d = profile_linear_params->acc;
            profile_linear_params->oldst= 1;
        } else {
            profile_linear_params->a_d = profile_linear_params->dec;
            profile_linear_params->oldst= 2;
        }
    } else if (profile_linear_params->u<0 && profile_linear_params->v_d>0) {
        /*initial and desired velocity - transition from -ve to +ve case*/
        if (profile_linear_params->oldst==-1) {
            profile_linear_params->a_d = -profile_linear_params->dec;
            profile_linear_params->oldst=-2;
        } else {
            profile_linear_params->a_d = -profile_linear_params->acc;
            profile_linear_params->oldst= -1;
        }
    }

    profile_linear_params->s_time = .001f;

    // compute time needed
    profile_linear_params->t = (profile_linear_params->v_d - profile_linear_params->u)/profile_linear_params->a_d;

    profile_linear_params->T = profile_linear_params->t/profile_linear_params->s_time;

    if(profile_linear_params->T<0)
        profile_linear_params->T = -profile_linear_params->T;

    //length = (int) round (T);
    profile_linear_params->s_time = profile_linear_params->t/profile_linear_params->T;

    return (int) round (profile_linear_params->T);
}

float __linear_profile_generate_float(int step, profile_linear_param *profile_linear_params)
{
    return  profile_linear_params->u + profile_linear_params->a_d * profile_linear_params->s_time * (float)step;
}
