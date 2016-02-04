/*
 * The copyrights, all other intellectual and industrial
 * property rights are retained by XMOS and/or its licensors.
 * Terms and conditions covering the use of this code can
 * be found in the Xmos End User License Agreement.
 *
 * Copyright XMOS Ltd 2010
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **/
#pragma once

#include <pwm_cli_common.h>

#include <xs1.h>

/**
 * @brief Update the PWM service with three new values
 *
 * On the next cycle through the PWM, the service will update the PWM
 * pulse widths with these new values
 *
 * @param ctrl Client control structure for this PWM server
 * @param c_pwm Channel for communication with the PWM service
 * @param value[] Array of values for the outputs of the PWM Service [0-13889 (max PWM value by default)]. (PWM period = 13889/250MHz = 55.55 us).
 */
void update_pwm_inv( t_pwm_control& ctrl, chanend c_pwm, unsigned value[]);

