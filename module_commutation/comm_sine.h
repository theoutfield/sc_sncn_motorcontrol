/*
 * comm_sine.h
 *
 *  Created on: 11.05.2012
 *      Author: mschwarz
 */

#pragma once

void comm_sine_init(chanend c_pwm_ctrl);

void comm_sine(chanend adc, chanend c_commutation, chanend c_hall, chanend c_pwm_ctrl);

void commutation(chanend c_adc, chanend  c_commutation,  chanend c_hall, chanend c_pwm_ctrl);
