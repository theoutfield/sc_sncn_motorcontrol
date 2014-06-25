
/**
 * @file profile_velocity.c
 * @brief Profile Generation for Velocity
 *      Implements velocity profile based on linear functions.
 * @author Pavan Kanajar <pkanajar@synapticon.com>
*/


#include <profile.h>
#include <stdio.h>

struct
{
    float max_acceleration, max_deceleration;  // max allowed acceleration & deceleration
    float acc, dec;                            // acceleration & deceleration input
    float u;                                   // initial velocity
    float v_d;                                 // desired velocity
    float a_d;                                 // desired acceleration
    float t;                                   // time
    float T;                                   // total no. of Samples
    float s_time;                              // sampling time
    int oldst;                                 // old state of acc/dec
} profile_velocity_params;

int init_velocity_profile(int target_velocity, int actual_velocity, int acceleration,
                          int deceleration, int max_velocity)
{
    profile_velocity_params.u = (float) actual_velocity;
    //profile_velocity_params.v_d = (float) target_velocity;
    profile_velocity_params.acc = (float) acceleration;
    profile_velocity_params.dec = (float) deceleration;

    if (target_velocity >= max_velocity) {
        target_velocity = max_velocity;
    } else if (target_velocity <= -max_velocity) {
        target_velocity = 0-max_velocity;
    }
    profile_velocity_params.v_d = (float) target_velocity;
    //printf("srta");
    //printf("\n%d\n",target_velocity);
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

int velocity_profile_generate(int step)
{
    return (int) round( profile_velocity_params.u + profile_velocity_params.a_d * profile_velocity_params.s_time * step);
}


int __initialize_velocity_profile(int target_velocity, int actual_velocity, int acceleration, \
                                  int deceleration, int max_velocity, profile_velocity_param *profile_velocity_params)
{
    profile_velocity_params->u = (float) actual_velocity;
    //profile_velocity_params->v_d = (float) target_velocity;
    profile_velocity_params->acc = (float) acceleration;
    profile_velocity_params->dec = (float) deceleration;

    if(target_velocity >= max_velocity) {
        target_velocity = max_velocity;
    } else if(target_velocity <= -max_velocity) {
        target_velocity = 0-max_velocity;
    }
    profile_velocity_params->v_d = (float) target_velocity;
    //printf("srta");
    //printf("\n%d\n",target_velocity);

    if (profile_velocity_params->u>=0 && profile_velocity_params->v_d >=0) {
        /* both initial and desired velocity - positive case */
        if (profile_velocity_params->v_d >= profile_velocity_params->u) {
            profile_velocity_params->a_d = profile_velocity_params->acc;
            profile_velocity_params->oldst= 1;
        } else if (profile_velocity_params->v_d < profile_velocity_params->u) {
            profile_velocity_params->a_d = profile_velocity_params->dec;
            profile_velocity_params->oldst= 2;
        }
    } else if(profile_velocity_params->u<=0 && profile_velocity_params->v_d <=0) {
        /*both initial and desired velocity - negative case*/
        if (profile_velocity_params->u==0) {
            profile_velocity_params->a_d = profile_velocity_params->acc;
            profile_velocity_params->oldst= -1;
        } else if (profile_velocity_params->v_d==0) {
            profile_velocity_params->a_d = -profile_velocity_params->dec;
            profile_velocity_params->oldst= -2;
        } else {
            if (profile_velocity_params->v_d < profile_velocity_params->u) {
                profile_velocity_params->a_d = -profile_velocity_params->acc;
                profile_velocity_params->oldst= -1;
            } else if (profile_velocity_params->v_d > profile_velocity_params->u) {
                profile_velocity_params->a_d = -profile_velocity_params->dec;
                profile_velocity_params->oldst= -2;
            }
        }
    } else if (profile_velocity_params->u>0 && profile_velocity_params->v_d<0) {
        /*initial and desired velocity - transition from +ve to -ve case*/
        if (profile_velocity_params->oldst==2 || profile_velocity_params->oldst==-2) {
            profile_velocity_params->a_d = profile_velocity_params->acc;
            profile_velocity_params->oldst= 1;
        } else {
            profile_velocity_params->a_d = profile_velocity_params->dec;
            profile_velocity_params->oldst= 2;
        }
    } else if (profile_velocity_params->u<0 && profile_velocity_params->v_d>0) {
        /*initial and desired velocity - transition from -ve to +ve case*/
        if (profile_velocity_params->oldst==-1) {
            profile_velocity_params->a_d = -profile_velocity_params->dec;
            profile_velocity_params->oldst=-2;
        } else {
            profile_velocity_params->a_d = -profile_velocity_params->acc;
            profile_velocity_params->oldst= -1;
        }
    }

    profile_velocity_params->s_time = .001f;

    // compute time needed
    profile_velocity_params->t = (profile_velocity_params->v_d - profile_velocity_params->u)/profile_velocity_params->a_d;

    profile_velocity_params->T = profile_velocity_params->t/profile_velocity_params->s_time;

    if(profile_velocity_params->T<0) {
        profile_velocity_params->T = -profile_velocity_params->T;
    }

    //length = (int) round (T);
    profile_velocity_params->s_time = profile_velocity_params->t/profile_velocity_params->T;

    return (int) round (profile_velocity_params->T);
}

int __velocity_profile_generate_in_steps(int step, profile_velocity_param *profile_velocity_params)
{
    return (int) round( profile_velocity_params->u + profile_velocity_params->a_d * profile_velocity_params->s_time * step);
}

