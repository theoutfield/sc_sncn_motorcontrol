
/**
 * \file adc_server_ad7949.h
 * \brief ADC Server
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \author Ludwig Orgler <lorgler@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */
/*
 * Copyright (c) 2014, Synapticon GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Execution of this software or parts of it exclusively takes place on hardware
 *    produced by Synapticon GmbH.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Synapticon GmbH.
 *
 */

#pragma once
#include <xs1.h>
#include <xclib.h>
#include "adc_common.h"

/**
 * \brief Non triggered ADC server
 *
 *
 * This server should be used if the somanet node is not used for motor
 * drive/control. This is the interface to AD7949 ADC devices. It controls
 * two devices so that two channels can be sampled simultaneously.
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
 * This server should be used if the somanet node is used for motor
 * drive/control. This is the interface to AD7949 ADC devices.
 * It controls two devices so that two channels can be sampled
 * simultaneously.
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
