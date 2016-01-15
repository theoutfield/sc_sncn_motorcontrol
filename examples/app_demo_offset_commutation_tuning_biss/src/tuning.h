/*
 * tuning.h
 *
 *  Created on: Jul 13, 2015
 *      Author: Synapticon GmbH
 */


#ifndef TUNING_H_
#define TUNING_H_

#include <platform.h>
#include <motorcontrol_service.h>
#include <pwm_service.h>
#include <refclk.h>

#include <xscope.h>
//#include <bldc_motor_config.h>
#include <mc_internal_constants.h>

void set_commutation_offset_clk(chanend c_signal, unsigned offset);

void set_commutation_offset_cclk(chanend c_signal, unsigned offset);

void run_offset_tuning(int input_voltage, interface MotorcontrolInterface client i_commutation, interface BISSInterface client ?i_biss);

#endif /* TUNING_H_ */
