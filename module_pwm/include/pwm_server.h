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

#include <pwm_ports.h>
#include <motor_control_interfaces.h>

/**
 * @brief Structure type to define the ports to manage the FET-driver in your IFM SOMANET device (if applicable).
 */
typedef struct {
    port ?p_coast;  /**< [Nullable] Port for management signals. */
    out port ?p_esf_rst_pwml_pwmh; /**< [Nullable] 4-bit Port to  enabling operation signals (if applicable in your SOMANET device). */
    port ?p_ff1; /**< [Nullable] Port to read out faults (if applicable in your SOMANET device). */
    port ?p_ff2; /**< [Nullable] Port to read out faults (if applicable in your SOMANET device). */
} FetDriverPorts;

/** Structure containing pwm server control data */
typedef struct PWM_SERV_TAG
{
	int id;
	unsigned ref_time;
	int data_ready; //Data ready flag
} PWM_SERV_TYP;

void predriver(FetDriverPorts &fet_driver_ports);
void pwm_config(PwmPorts &ports);
void pwm_check(PwmPorts &ports);
void pwm_service_task( // Implementation of the Centre-aligned, High-Low pair, PWM server, with ADC synchronization
        unsigned motor_id, // Motor identifier
        PwmPorts &ports,
        server interface update_pwm i_update_pwm,
        int duty_start_brake,
        int duty_maintain_brake,
        int time_start_brake,
        int ifm_tile_usec
);



#endif // _PWM_SERVER_H_
