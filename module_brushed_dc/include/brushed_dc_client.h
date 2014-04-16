
/**
 * \file
 * \brief Brushed Motor Drive Client function
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 0.9beta
 * \date 10/04/2014
 */

#pragma once

#include <pwm_config.h>
#include "pwm_cli_inv.h"
#include "a4935.h"
#include "hall_client.h"
#include <bldc_motor_config.h>


/**
 *  \brief Set Input voltage for brushed dc motor
 *
 *   Output
 * 	\channel c_voltage channel to send out motor voltage input value
 *
 * 	 Input
 * 	\param input_voltage is motor voltage input value to be set range allowed [-(CONTROL_LIMIT_PWM-150) to (CONTROL_LIMIT_PWM+150)]
 */
void set_bdc_voltage(chanend c_voltage, int input_voltage);

