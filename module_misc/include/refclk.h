/**
 * @file refclk.h
 * @brief Reference Clock definitions
 * @author Synapticon GmbH <support@synapticon.com>
 */


#pragma once

/**
 * Defines the amount of ticks of a 100MHz timer (standard system reference frequency) for 1 us.
 */
#define USEC_STD       100

/**
 * Defines the amount of ticks of a 100MHz timer (standard system reference frequency) for 1 ms.
 */
#define MSEC_STD    100000

/**
 * Defines the amount of ticks of a 100MHz timer (standard system reference frequency) for 1 s.
 */
#define SEC_STD   100000000



/**
 * Defines the amount of ticks of a 250MHz timer (Motorcontrol TILE reference frequency) for 1 us.
 */
#define USEC_FAST       250

/**
 * Defines the amount of ticks of a 250MHz timer (Motorcontrol TILE reference frequency) for 1 ms.
 */

#define MSEC_FAST    250000

/**
 * Defines the amount of ticks of a 250MHz timer (Motorcontrol TILE reference frequency) for 1 s.
 */
#define SEC_FAST  250000000
