/**
 * @file adc_ad7949.h
 * @brief ADC Server
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <xs1.h>
#include <xclib.h>
#include <adc_service.h>

/**
 * @brief Service to sample analogue inputs of ADC module
 *
 * @param iADC[2]               Interface to communicate with clients and send the measured values
 * @param adc_ports             Structure type to manage the AD7949 ADC chip.
 * @param current_sensor_config Structure type to calculate the proper sign (positive/negative) of sampled phase currents
 * @param i_watchdog            Interface to communicate with watchdog service
 * @param operational_mode      Reserved
 *
 * @return void
 */
void adc_ad7949(
        interface ADCInterface server iADC[2],
        AD7949Ports &adc_ports,
        CurrentSensorsConfig &current_sensor_config,
        interface WatchdogInterface client ?i_watchdog, int operational_mode);
