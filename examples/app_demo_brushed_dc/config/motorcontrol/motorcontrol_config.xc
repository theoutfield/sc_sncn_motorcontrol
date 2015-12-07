/*
 * commutation_config.xc
 *
 *  Created on: Nov 30, 2015
 *      Author: atena
 */

#include <motorcontrol_config.h>

void init_motorcontrol_config(MotorcontrolConfig & commutation_params)
{
    commutation_params.motor_type = BDC_MOTOR;
    commutation_params.angle_variance = (60 * 4096) / (POLE_PAIRS * 2 * 360);

    if (POLE_PAIRS < 4) {
        commutation_params.nominal_speed =  MAX_NOMINAL_SPEED * 4;
    } else if (POLE_PAIRS >= 4) {
        commutation_params.nominal_speed =  MAX_NOMINAL_SPEED;
    }

    commutation_params.commutation_loop_freq =  COMMUTATION_LOOP_FREQUENCY_KHZ;

}
