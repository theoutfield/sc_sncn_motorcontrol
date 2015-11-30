/*
 * control_config.xc
 *
 *  Created on: Nov 30, 2015
 *      Author: atena
 */
#include <refclk.h>
#include <control_config.h>
#include <bldc_motor_config.h>


void init_position_control_config(ControlConfig &position_ctrl_params){

    position_ctrl_params.Kp_n = POSITION_Kp_NUMERATOR;
    position_ctrl_params.Kp_d = POSITION_Kp_DENOMINATOR;
    position_ctrl_params.Ki_n = POSITION_Ki_NUMERATOR;
    position_ctrl_params.Ki_d = POSITION_Ki_DENOMINATOR;
    position_ctrl_params.Kd_n = POSITION_Kd_NUMERATOR;
    position_ctrl_params.Kd_d = POSITION_Kd_DENOMINATOR;
    position_ctrl_params.Loop_time = 1 * MSEC_STD; // units - for CORE 2/1/0 only default

    position_ctrl_params.Control_limit = BLDC_PWM_CONTROL_LIMIT; // PWM resolution

    if(position_ctrl_params.Ki_n != 0) // auto calculated using control_limit
    {
        position_ctrl_params.Integral_limit = position_ctrl_params.Control_limit * (position_ctrl_params.Ki_d/position_ctrl_params.Ki_n);
    } else {
        position_ctrl_params.Integral_limit = 0;
    }

    position_ctrl_params.sensor_used = SENSOR_USED; // units - for CORE 2/1/0 only default

    return;

}
