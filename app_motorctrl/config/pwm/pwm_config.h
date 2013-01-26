/**
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

// Define dead time period in 10ns period, i.e. dead time = PWM_DEAD_TIME * 10ns
//#define PWM_DEAD_TIME 120
// For 250 MHz Reference Clock: dead time = PWM_DEAD_TIME * 4ns
#define PWM_DEAD_TIME 300
// we have t_rise = t_fall = 100ns -- set to 4*30=120ns
//#define PWM_DEAD_TIME 30

// Define the resolution of PWM (affects operational freq. as tied to ref clock)
// pwm-frequency = refclk / PWM_MAX_VALUE
//#define PWM_MAX_VALUE 4096 /* 61,035 kHz (250 MHz refclk) */
//#define PWM_MAX_VALUE 8192
//#define PWM_MAX_VALUE 10000 /* 25 kHz (250 MHz refclk) */
//#define PWM_MAX_VALUE 12500 /* 20 kHz */
#define PWM_MAX_VALUE 13889 /* 18 kHz */

#define PWM_MIN_LIMIT 250
#define PWM_MAX_LIMIT (PWM_MAX_VALUE - PWM_DEAD_TIME)

// default limits
//#define PWM_MIN_LIMIT 200
//#define PWM_MAX_LIMIT 3800

// pwm limits for PWM_MAX_VALUE==4096 && PWM_DEAD_TIME==120
//#define PWM_MIN_LIMIT 98
//#define PWM_MAX_LIMIT 3985

// pwm limits for PWM_MAX_VALUE==4096 && PWM_DEAD_TIME==300
//#define PWM_MIN_LIMIT 54
//#define PWM_MAX_LIMIT 3805

// Define if ADC sampling is locked to PWM switching. The ADC sampling will occur in the middle of the  switching sequence.
// It is triggered over a channel. Set this define to 0 to disable this feature
#define LOCK_ADC_TO_PWM 1
