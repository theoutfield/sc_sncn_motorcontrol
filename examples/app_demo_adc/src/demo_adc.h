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
 * @brief Client demo to show how AD7265 can be used.
 * It sets the analogue input channel (to be sampled by ADC), and recieves the
 * converted digital value from adc server. This client is recommended to be used with its corresponding service demo.
 *
 * @param i_adc     Interface to communicate data with the server demo
 *
 * @return void
 */
void adc7265_client_demo(interface ADCInterface client i_adc);

/**
 * @brief Client demo to show how AD7949 can be used.
 * It sets the analogue input channel (to be sampled by ADC), and recieves the
 * converted digital value from adc server. This client is recommended to be used with its corresponding service demo.
 *
 * @param i_adc     Interface to communicate data with the server demo
 *
 * @return void
 */
void adc7949_client_demo(interface ADCInterface client i_adc);
