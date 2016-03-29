/*
 * profile_torque_internal.h
 *
 *  Created on: Mar 28, 2016
 *      Author: Synapticon
 */

typedef struct
{
    float max_acceleration, max_deceleration;   // max allowed acceleration & deceleration
    float max_value;                            // max allowed value
    int polarity;
    float acc, dec;                             // acceleration & deceleration input
    float u;                                    // initial value
    float v_d;                                  // desired value
    float a_d;                                  // desired acceleration
    float t;                                    // time
    float T;                                    // total no. of Samples
    float s_time;                               // sampling time
    int oldst;                                  // old state of acc/dec
} ProfileLinearParams;

/**
 * @brief Calculate the number of steps for torque profile
 *
 * @param profile_linear_params all parameters needed to do the calculations
 *
 * @return no. of steps for linear profile : range [1 - steps]
 */
int calculate_profile_linear_steps(ProfileLinearParams & profile_linear_params);

/**
 * @brief Generate Linear Profile
 *
 * @param profile_linear_params all parameters needed to do the calculations
 * @param step current step of the profile
 *
 * @return corresponding target value at the step input
 */
int generate_profile_step_torque(ProfileLinearParams profile_linear_params, int step);
