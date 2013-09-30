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
 * \brief Client functions for non triggered adc server
 *
 * \channel c_adc the channel for communicating with the adc server
 *
 * \return Ia phase current value (raw) in whatever format the ADC delivers them in
 * \return Ib phase current value (raw) in whatever format the ADC delivers them in
 */
{int, int} get_adc_external_potentiometer(chanend c_adc);

/**
 * \brief Calibrated Phase currents from the non triggered ADC server
 *
 * \channel c_adc the channel for communicating with the adc server
 *
 * \return Ia calibrated phase current value
 * \return Ib calibrated phase current value
 * \return Calibration constant for phase current Ia
 * \return Calibration constant for phase current Ib
 * \return ADC temperature sensor 1 value
 * \return ADC temperature sensor 2 value
 * \return ADC Voltage Supply value
 * \return ADC dummy value
 * \return External input 1 value
 * \return External input 2 value
 */
{int, int, int, int, int, int, int, int, int, int} get_adc_calibrated_ad7949( chanend c_adc);



/**
 * \brief ADC calibration sequence for Triggered ADC server
 *
 * \channel c_adc the channel for communicating with the adc server
 *
 */
void do_adc_calibration_ad7949( chanend c_adc );

/**
 * \brief Calibrated current for two phases from the Triggered ADC server
 *
 * \channel c_adc the channel for communicating with the adc server
 *
 * \return Ia calibrated phase current value
 * \return Ib calibrated phase current value
 *
 */
{int, int} get_adc_vals_calibrated_int16_ad7949( chanend c_adc );


