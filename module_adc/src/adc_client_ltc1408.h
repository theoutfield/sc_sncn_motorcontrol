/**
 * Module:  module_dsc_adc
 * Version: 1v0alpha2
 * Build:   60a90cca6296c0154ccc44e1375cc3966292f74e
 * File:    adc_client.h
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

#pragma once

/**
 * \brief ADC current calibration sequence
 *
 * \channel c_adc the channel for communicating with the adc server
 *
 */
void do_adc_calibration_ltc1408( chanend c_adc );

/**
 * \brief Client functions for triggered adc server
 *
 * \channel c_adc the channel for communicating with the adc server
 *
 * \return Ia phase current value (raw) in whatever format the ADC delivers them in
 * \return Ib phase current value (raw) in whatever format the ADC delivers them in
 * \return Ic phase current value (raw) in whatever format the ADC delivers them in
 *
 */
{unsigned, unsigned, unsigned} get_adc_vals_ltc1408( chanend c_adc );

/* get calibrated three phase current from the adc*/

/**
 * \brief Calibrated current for three phases from the adc
 *
 * \channel c_adc the channel for communicating with the adc server
 *
 * \return Ia calibrated phase current value
 * \return Ib calibrated phase current value
 * \return Ic calibrated phase current value
 *
 */
{int, int, int} get_adc_calibrated_current_ltc1408( chanend c_adc );


