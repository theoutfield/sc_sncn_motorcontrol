/**
 * \file adc_client_ad7949.h
 *
 * ADC Client for DC 900
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

/**
 * The client library function for adc server
 *
 * \channels:
 * 			c_adc -	the channel for communicating with the adc server
 */

/*
 * client functions for non triggered adc
 */

/* get raw two phase current values in whatever format the ADC delivers them in */
{unsigned, unsigned} get_adc_vals_raw_ad7949( chanend c_adc );

/*
 *
 *	function returns the following parameters respectively
 *
 *	phase currents				: 	Ia and Ib
 *	phase calibration values	: 	I_calib.Ia_calib and I_calib.Ib_calib
 *	Temperature values			:  	adc_Temperature1, adc_Temperature2
 *					  			: 	adc_VoltageSupply, adc_Dummy
 *  External Sensor				: 	ExternalPot1, ExternalPot2
 */
{int, int, int, int, int, int, int, int, int, int} get_adc_calibrated_ad7949( chanend c_adc, int iPwmOnOff);


/*
 * client functions for triggered adc
 */

/* get calibrated current for two phases from the adc*/
{int, int} get_adc_vals_calibrated_int16_ad7949( chanend c_adc );

/* ADC calibration sequence */
void do_adc_calibration_ad7949( chanend c_adc );
