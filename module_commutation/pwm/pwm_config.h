/**************************************************************************
 * \file dc_motor_config.h
 *	PWM config file
 *
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Authors:  Martin Schwarz <mschwarz@synapticon.com>
 *
 * All code contained in this package under Synapticon copyright must be
 * licensing for any use from Synapticon. Please contact support@synapticon.com for
 * details of licensing.
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **************************************************************************/
#pragma once

// Define dead time period in 10ns period, i.e. dead time = PWM_DEAD_TIME * 10ns
// For 250 MHz Reference Clock: dead time = PWM_DEAD_TIME * 4ns
#define PWM_DEAD_TIME 300
#define PWM_MAX_VALUE 13889 /* 18 kHz */
#define PWM_MIN_LIMIT 250
#define PWM_MAX_LIMIT (PWM_MAX_VALUE - PWM_DEAD_TIME)
// Define if ADC sampling is locked to PWM switching. The ADC sampling will occur in the middle of the  switching sequence.
// It is triggered over a channel. Set this define to 0 to disable this feature
#define LOCK_ADC_TO_PWM 1
