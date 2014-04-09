
/**
 * \file adc_client_ad7949.h
 * \brief ADC Client
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
#include <adc_common.h>

/**
 * \brief Get external analog sensor value
 *
 * \channel c_adc the channel for communicating with the adc server
 *
 * \return External analog input 1 value (raw range: 0 - 16383)
 * \return External analog input 2 value (raw range: 0 - 16383)
 */
{int, int} get_adc_external_ad7949(chanend c_adc);


/**
 * \brief Get all ADC values
 *
 * \channel c_adc the channel for communicating with the adc server
 *
 * \return Ia phase current value (raw)
 * \return Ib phase current value (raw)
 * \return ADC temperature sensor 1 value (raw)
 * \return ADC temperature sensor 2 value (raw)
 * \return ADC Voltage Supply value
 * \return ADC dummy value
 * \return External potentiometer input 1 value (raw)
 * \return External potentiometer input 2 value (raw)
 */
{int, int, int, int, int, int, int, int} get_adc_all_ad7949( chanend c_adc);


/**
 * \brief ADC current calibration sequence for Triggered ADC server
 *
 * \channel c_adc the channel for communicating with the adc server
 *
 */
void do_adc_calibration_ad7949(chanend c_adc, calib_data &I_calib);

/**
 * \brief Get Calibrated current of two phases from the Triggered ADC server
 *
 * \channel c_adc the channel for communicating with the adc server
 *
 * \return Ia calibrated phase current value
 * \return Ib calibrated phase current value
 *
 */
{int, int} get_adc_calibrated_current_ad7949(chanend c_adc, calib_data &I_calib);


