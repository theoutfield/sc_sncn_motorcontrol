/**
 * \file adc_ltc1408.h
 *
 * ADC Server for DC 100
 *
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

#include <xs1.h>
#include <xclib.h>
#include "adc_common.h"

/**
 * \brief Triggered ADC server
 *
 * This is the server thread implementation for the LTC1408 ADC device.
 *
 * \channel	c_adc channel to receive ADC output
 * \channel c_trig channel to trigger adc from the PWM modules
 * \clock clk clock for the ADC device serial port
 *
 * \port SCLK port which feeds the ADC serial clock
 * \port CNVST port for ADC convert strobe
 * \port DATA port for ADC data
 *
 */
void adc_ltc1408_triggered( chanend c_adc, chanend c_trig, clock clk, port out SCLK, buffered out port:32 CNVST, in buffered port:32 DATA);
