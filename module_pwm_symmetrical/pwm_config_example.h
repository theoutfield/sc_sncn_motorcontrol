/* PWM config file as expected by module_pwm_symmetrical */

#pragma once

/* All timing values depend on reference clock setting. For 100 MHz reference clock each tick
 * is equivalent to 10ns (for 250 MHz clock it's 4ns)
 */
//#define PWM_DEAD_TIME 300

/* This defines the PWM resolution. Because the PWM is driven by a fixed clock,
 * this also configures the PWM frequency at the same time. The relation is as follows:
 * PWM_MAX_VALUE = PWM_CLOCK / PWM_FREQUENCY (e.g. 250 MHz / 18 kHz = 13889)
 */
//#define PWM_MAX_VALUE 13889
