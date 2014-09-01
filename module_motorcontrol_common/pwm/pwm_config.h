/**
 * @file pwm_config.h
 * @brief PWM configuration file
 * @author Martin Schwarz <mschwarz@synapticon.com>
*/

#pragma once

// Define dead time period in 10ns period, i.e. dead time = PWM_DEAD_TIME * 10ns
// For 250 MHz Reference Clock: dead time = PWM_DEAD_TIME * 4ns
#define PWM_DEAD_TIME 300
#define PWM_MAX_VALUE 13889 /* 18 kHz */
#define PWM_MIN_LIMIT 250   /* FIXME: is this still used? */
#define PWM_MAX_LIMIT (PWM_MAX_VALUE - PWM_DEAD_TIME)

// Define if ADC sampling is locked to PWM switching. The ADC sampling will occur
// in the middle of the  switching sequence.
// It is triggered over a channel. Set this define to 0 to disable this feature
#define LOCK_ADC_TO_PWM 1
