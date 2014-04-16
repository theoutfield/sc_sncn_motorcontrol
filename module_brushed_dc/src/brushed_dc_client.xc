
/**
 * \file
 * \brief Brushed Motor Drive Client function
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 0.9beta
 * \date 10/04/2014
 */


#include "brushed_dc_client.h"
#include <xs1.h>
#include <bldc_motor_config.h>
#include "refclk.h"
#include <internal_config.h>



/* MAX Input value CONTROL_LIMIT_PWM */
void set_bdc_voltage(chanend c_voltage, int input_voltage)
{
	c_voltage <: 2;
	c_voltage <: input_voltage;
	return;
}


