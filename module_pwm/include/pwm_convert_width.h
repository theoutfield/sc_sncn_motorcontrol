/*
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
#ifndef _PWM_CONVERT_WIDTH_H_
#define _PWM_CONVERT_WIDTH_H_


#include <xclib.h>
#include <xccompat.h>

#include <xs1.h>
#include <assert.h>
#include <xccompat.h>

#include "pwm_general.h"

/**
 * @brief Converts PWM structure reference to address.
 *
 * @param pwm_ps Pointer to PWM control structure
 * @return Address
 */
unsigned long get_pwm_struct_address( // Converts PWM structure reference to address
	REFERENCE_PARAM( PWM_ARRAY_TYP ,pwm_ps ) // Pointer to PWM structure containing array of buffers
); // Return address

/**
 * @brief Convert all PWM pulse widths to pattern/time_offset port data
 *
 * @param pwm_comms_ps      Pointer to structure containing PWM communication data
 * @param pwm_buf_ps        Pointer to Structure containing buffered PWM output data
 * @param pwm_max_value     Maximum pwm value which can be sent to pwm server (number of clock ticks)
 * @param pwm_deadtime      Number of clock ticks in over deadtime period
 *
 * @return void
 */
void convert_all_pulse_widths(
	REFERENCE_PARAM( PWM_COMMS_TYP ,pwm_comms_ps),
	REFERENCE_PARAM( PWM_BUFFER_TYP ,pwm_buf_ps),
	unsigned int pwm_max_value,
	unsigned int pwm_deadtime
);

/**
 * @brief Converts PWM Pulse-width to port data in shared memory
 *
 * @param pwm_comms_ps      Pointer to structure containing PWM communication data
 * @param pwm_max_value     Maximum pwm value which can be sent to pwm server (number of clock ticks)
 * @param pwm_deadtime      Number of clock ticks in over deadtime period
 *
 * @return void
 */
void convert_widths_in_shared_mem(
	REFERENCE_PARAM( PWM_COMMS_TYP ,pwm_comms_ps),
	unsigned int pwm_max_value,
	unsigned int pwm_deadtime
);

#endif /* _PWM_CONVERT_WIDTH_H_ */
