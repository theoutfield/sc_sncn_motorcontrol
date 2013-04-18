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

/** Triggered ADC server
 *
 * This is the server thread implementation for the LTC1408 ADC device.
 *
 * \channels:
 * 				c_adc  	-	channel to receive ADC output
 * 				c_trig 	- 	channel to trigger adc from the PWM modules
 * \clocks:
 * 				clk		- 	clock for the ADC device serial port
 * \ports:
 *        		SCLK 	-	port which feeds the ADC serial clock
 * 		  	 	CNVST 	-	ADC convert strobe
 * 				DATA 	-	ADC data port
 *
 */
void adc_ltc1408_triggered( chanend c_adc, chanend c_trig, clock clk, port out SCLK, buffered out port:32 CNVST, in buffered port:32 DATA);
