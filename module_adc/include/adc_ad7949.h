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
 * @brief Non triggered ADC server
 *
 *
 * This server should be used if the SOMANET node is not used for motor
 * drive/control. This is the interface to AD7949 ADC devices. It controls
 * two devices so that two channels can be sampled simultaneously.
 *
 * @param i_adc Interface to receive ADC output
 * @param adc_ports Structure containing hardware ports (e.g. SPI) to the ADC
 * @param current_sensor_config Structure containing configurations for this service
 *
 */
void adc_ad7949(
        interface ADCInterface server i_adc[2],
        AD7949Ports &adc_ports,
        CurrentSensorsConfig &current_sensor_config,
        interface WatchdogInterface client ?i_watchdog, int operational_mode);

void adc_ad7949_fixed_channel(
        interface ADCInterface server i_adc[2],
        AD7949Ports &adc_ports,
        CurrentSensorsConfig &current_sensor_config,
        interface WatchdogInterface client ?i_watchdog, int operational_mode);
