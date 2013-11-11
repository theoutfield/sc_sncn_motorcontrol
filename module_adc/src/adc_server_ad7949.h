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

/**
 * \brief Non triggered ADC server
 *
 * This is the interface to AD7949 ADC devices. It controls two devices
 * so that two channels can be sampled simultaneously.
 *
 * \channel c_adc channel to receive ADC output
 *
 * \clock clk clock for the ADC device serial port
 *
 * \port p_sclk_conv_mosib_mosia 4-bit port for ADC control interface
 * \port p_data_a 1-bit port for ADC data channel 0
 * \port p_data_b 1-bit port for ADC data channel 1
 *
 */
void adc_ad7949( chanend c_adc,
			   clock clk,
			   buffered out port:32 p_sclk_conv_mosib_mosia,
			   in buffered port:32 p_data_a,
			   in buffered port:32 p_data_b );


/**
 * \brief Triggered ADC server
 *
 * This is the interface to AD7949 ADC devices. It controls two devices
 * so that two channels can be sampled simultaneously.
 *
 * \channel	c_adc channel to receive ADC output
 * \channel	c_trig channel to trigger adc from the PWM modules
 *
 * \clock clk clock for the ADC device serial port
 *
 * \port p_sclk_conv_mosib_mosia 4-bit port for ADC control interface
 * \port p_data_a 1-bit port for ADC data channel 0
 * \port p_data_b 1-bit port for ADC data channel 1
 *
 */
void adc_ad7949_triggered( chanend c_adc,
			   	   	   	   chanend c_trig,
			   	   	   	   clock clk,
			   	   	   	   buffered out port:32 p_sclk_conv_mosib_mosia,
			   	   	   	   in buffered port:32 p_data_a,
			   	   	   	   in buffered port:32 p_data_b );
