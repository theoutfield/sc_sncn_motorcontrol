/*
 * control_config.xc
 *
 *  Created on: Nov 30, 2015
 *      Author: atena
 */
#include <refclk.h>
#include <control_config.h>
#include <user_config.h>


void init_torque_control_config(ControlConfig &torque_ctrl_params){

     torque_ctrl_params.Kp_n = TORQUE_Kp_NUMERATOR;
     torque_ctrl_params.Kp_d = TORQUE_Kp_DENOMINATOR;
     torque_ctrl_params.Ki_n = TORQUE_Ki_NUMERATOR;
     torque_ctrl_params.Ki_d = TORQUE_Ki_DENOMINATOR;
     torque_ctrl_params.Kd_n = TORQUE_Kd_NUMERATOR;
     torque_ctrl_params.Kd_d = TORQUE_Kd_DENOMINATOR;
     torque_ctrl_params.Loop_time = 1 * MSEC_STD; // units - for CORE 2/1/0 only default

     torque_ctrl_params.Control_limit = BLDC_PWM_CONTROL_LIMIT; // PWM resolution

     if(torque_ctrl_params.Ki_n != 0) {
         // auto calculated using control_limit
         torque_ctrl_params.Integral_limit = (torque_ctrl_params.Control_limit * torque_ctrl_params.Ki_d) / torque_ctrl_params.Ki_n;
     } else {
         torque_ctrl_params.Integral_limit = 0;
     }

    torque_ctrl_params.sensor_used = SENSOR_USED; // units - for CORE 2/1/0 only default

    return;

}
