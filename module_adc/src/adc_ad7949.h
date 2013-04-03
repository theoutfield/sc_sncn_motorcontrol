/**
 * \file adc_ad7949.h
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

/** \brief Execute the triggered ADC server
 *
 * This is the interface to AD7949 ADC devices. It controls two devices
 * so that two channels can be sampled simultaneously.
 *
 * \param c_adc the array of ADC control channels
 * \param c_trig the array of channels to receive triggers from the PWM modules
 * \param clk the clock for the ADC device serial port
 * \param p_sclk_conv_mosib_mosia 4-bit port, ADC control interface
 * \param p_data_a 1-bit port, ADC data channel 0
 * \param p_data_b 1-bit port, ADC data channel 1
 */
void adc_ad7949_triggered( chanend c_adc,
			   clock clk,
			   buffered out port:32 p_sclk_conv_mosib_mosia,
			   in buffered port:32 p_data_a,
			   in buffered port:32 p_data_b );

