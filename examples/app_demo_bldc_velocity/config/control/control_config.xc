/*
 * control_config.xc
 *
 *  Created on: Nov 30, 2015
 *      Author: atena
 */
#include <refclk.h>
#include <control_config.h>
#include <bldc_motor_config.h>


void init_velocity_control_config(ControlConfig &velocity_ctrl_params){

    velocity_ctrl_params.Kp_n = VELOCITY_Kp_NUMERATOR;
    velocity_ctrl_params.Kp_d = VELOCITY_Kp_DENOMINATOR;
    velocity_ctrl_params.Ki_n = VELOCITY_Ki_NUMERATOR;
    velocity_ctrl_params.Ki_d = VELOCITY_Ki_DENOMINATOR;
    velocity_ctrl_params.Kd_n = VELOCITY_Kd_NUMERATOR;
    velocity_ctrl_params.Kd_d = VELOCITY_Kd_DENOMINATOR;

    if (velocity_ctrl_params.Loop_time != MSEC_FAST)////FixMe: implement reference clock check
        velocity_ctrl_params.Loop_time = 1 * MSEC_STD; // units - core timer value //CORE 2/1/0 default

    if (MOTOR_TYPE == BDC) {
        velocity_ctrl_params.Control_limit = BDC_PWM_CONTROL_LIMIT; // PWM resolution
    }
    else {
        velocity_ctrl_params.Control_limit = BLDC_PWM_CONTROL_LIMIT; // PWM resolution
    }

    if(velocity_ctrl_params.Ki_n != 0) {
        // auto calculated using control_limit
        velocity_ctrl_params.Integral_limit = velocity_ctrl_params.Control_limit * (velocity_ctrl_params.Ki_d/velocity_ctrl_params.Ki_n) ;
    } else {
        velocity_ctrl_params.Integral_limit = 0;
    }

    velocity_ctrl_params.sensor_used = SENSOR_USED;

    return;

}
