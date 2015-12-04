/*
 * commutation_config.xc
 *
 *  Created on: Nov 30, 2015
 *      Author: atena
 */

#include <motorcontrol_config.h>

void init_commutation_config(MotorcontrolConfig & commutation_params)
{
    commutation_params.angle_variance = (60 * 4096) / (POLE_PAIRS * 2 * 360);

    if (POLE_PAIRS < 4) {
        commutation_params.nominal_speed =  MAX_NOMINAL_SPEED * 4;
    } else if (POLE_PAIRS >= 4) {
        commutation_params.nominal_speed =  MAX_NOMINAL_SPEED;
    }

    commutation_params.commutation_loop_freq =  COMMUTATION_LOOP_FREQUENCY_KHZ;
    commutation_params.hall_offset_clk =  COMMUTATION_OFFSET_CLK;
    commutation_params.hall_offset_cclk = COMMUTATION_OFFSET_CCLK;
    commutation_params.bldc_winding_type = WINDING_TYPE;
    commutation_params.qei_forward_offset = 0;
    commutation_params.qei_backward_offset = 0;
}
