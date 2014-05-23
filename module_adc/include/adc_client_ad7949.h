/** 
 * \file adc_client_ad7949.h
 * \brief ADC Client
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \author Ludwig Orgler <lorgler@synapticon.com>
 */

#pragma once

#include <adc_common.h>


/**
 * \fn {int, int} get_adc_external_ad7949(chanend c_adc)
 * 
 * \brief Get external analog sensor value 
 *
 * \param c_adc Channel for communicating with the adc server
 *
 * \return External analog input 1 value (raw range: 0 - 16383)
 * \return External analog input 2 value (raw range: 0 - 16383)
 */
{int, int} get_adc_external_ad7949(chanend c_adc);


/**
 * \fn {int, int, int, int, int, int, int, int} get_adc_all_ad7949( chanend c_adc)
 * \brief Get all ADC values
 *
 * \param c_adc Channel for communicating with the adc server
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
 * \param c_adc the channel for communicating with the adc server
 *
 */
void do_adc_calibration_ad7949(chanend c_adc, calib_data &I_calib);

/**
 * \fn {int, int} get_adc_calibrated_current_ad7949( chanend c_adc )
 * \brief Get Calibrated current of two phases from the Triggered ADC server
 *
 * \param c_adc the channel for communicating with the adc server
 *
 * \return Ia calibrated phase current value
 * \return Ib calibrated phase current value
 *
 */
{int, int} get_adc_calibrated_current_ad7949(chanend c_adc, calib_data &I_calib);


