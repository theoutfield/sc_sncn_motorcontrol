/*
 * demo_adc.h
 *
 *  Created on: Jul 13, 20117
 *      Author: Synapticon GmbH
 */

#pragma once

#include <motor_control_interfaces.h>
#include <adc_service.h>

/**
 * @brief Client demo to show how analogue to digital converter can be used.
 * It sets the analogue input channel (to be sampled by ADC), and recieves the
 * converted digital value from adc server.
 *
 * @param i_adc     Interface to communicate data with the server demo
 * @param i_adc     ADC type (AD_7949 or AD_7265)
 *
 * @return void
 */
void adc_client_demo(interface ADCInterface client i_adc, int adc_type);

