/*
 * The copyrights, all other intellectual and industrial
 * property rights are retained by XMOS and/or its licensors.
 * Terms and conditions covering the use of this code can
 * be found in the Xmos End User License Agreement.
 *
 * Copyright XMOS Ltd 2013
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **/

#ifndef _PWM_SERVER_H_
#define _PWM_SERVER_H_


#include <stdio.h>
#include <stdlib.h>

#include <xs1.h>
#include <assert.h>
#include <print.h>

#include "app_global.h"
#include <control_variables.h>

#include "pwm_general.h"
#include "pwm_convert_width.h"
#include <pwm_ports.h>

/** Structure containing pwm server control data */
typedef struct PWM_SERV_TAG
{
	int id;
	unsigned ref_time;
	int data_ready; //Data ready flag
} PWM_SERV_TYP;


interface update_pwm
{
    void update_server_control_data(PWM_ARRAY_TYP pwm_ctrl_s, int pwm_test, int pwm_on, int break_active);
};

void pwm_config(PwmPorts &ports);


void update_pwm(control_variables& cv, PWM_COMMS_TYP& pwm_comms_s);
void pwm_check(PwmPorts &ports);
void pwm_service_task( // Implementation of the Centre-aligned, High-Low pair, PWM server, with ADC synchronization
    unsigned motor_id, // Motor identifier
    PwmPorts &ports,
    server interface update_pwm i_update_pwm
);

#endif // _PWM_SERVER_H_
