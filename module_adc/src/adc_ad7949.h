/**
 * \file adc_ad7949.h
 *
 *  ADC Server for DC 900
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Author: Martin Schwarz <mschwarz@synapticon.com>
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the 
 * copyright notice above.
 *
 **/

#pragma once
#include <xs1.h>
#include <xclib.h>
#include "adc_common.h"

/** Non triggered ADC server
 *
 * This is the interface to AD7949 ADC devices. It controls two devices
 * so that two channels can be sampled simultaneously.
 *
 * \channels:
 * 				c_adc  -	channel to receive ADC output
 * \clocks:
 * 				clk    - 	clock for the ADC device serial port
 * \ports:
 *        		p_sclk_conv_mosib_mosia -	4-bit port, ADC control interface
 * 		  	 	p_data_a 				-	1-bit port, ADC data channel 0
 * 				p_data_b 				-	1-bit port, ADC data channel 1
 *
 */
void adc_ad7949( chanend c_adc,
			   clock clk,
			   buffered out port:32 p_sclk_conv_mosib_mosia,
			   in buffered port:32 p_data_a,
			   in buffered port:32 p_data_b );


/** Triggered ADC server
 *
 * This is the interface to AD7949 ADC devices. It controls two devices
 * so that two channels can be sampled simultaneously.
 *
 * \channels:
 * 				c_adc  -	channel to receive ADC output
 * 				c_trig - 	channel to trigger adc from the PWM modules
 * \clocks:
 * 				clk    - 	clock for the ADC device serial port
 * \ports:
 *        		p_sclk_conv_mosib_mosia -	4-bit port, ADC control interface
 * 		  	 	p_data_a 				-	1-bit port, ADC data channel 0
 * 				p_data_b 				-	1-bit port, ADC data channel 1
 *
 */
void adc_ad7949_triggered( chanend c_adc,
			   	   	   	   chanend c_trig,
			   	   	   	   clock clk,
			   	   	   	   buffered out port:32 p_sclk_conv_mosib_mosia,
			   	   	   	   in buffered port:32 p_data_a,
			   	   	   	   in buffered port:32 p_data_b );
