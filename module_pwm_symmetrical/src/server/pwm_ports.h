/*
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

/**
 * @brief Structure type for PWM ports.
 */
typedef struct{
    buffered out port:32 p_pwm[3]; /**< Array of 3 ports for the high-side PWM signals output */
    buffered out port:32 p_pwm_inv[3]; /**< Array of 3 ports for the low-side PWM signals output */
    buffered out port:32 ?p_pwm_phase_d; /**< Port for the high-side PWM signal output of an optional 4th phase */
    buffered out port:32 ?p_pwm_phase_d_inv; /**< Port for the low-side PWM signal output of an optional 4th phase */
    clock clk; /**< Hardware clock used as time reference */
    in port ?dummy_port; /**< Any unused port. Used internally for accurate timming */
} PwmPorts;

