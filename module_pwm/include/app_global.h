/**
 * The copyrights, all other intellectual and industrial
 * property rights are retained by XMOS and/or its licensors.
 * Terms and conditions covering the use of this code can
 * be found in the Xmos End User License Agreement.
 *
 * Copyright XMOS Ltd 2013
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 **/

#ifndef _APP_GLOBAL_H_
#define _APP_GLOBAL_H_

/**
 * @brief Define number of connected motors (1 or 2).
 */
#define _NUMBER_OF_MOTORS 1

/**
 * @brief Define Motor Identifier (0 or 1)
 */
#define _MOTOR_ID 0

/**
 * @brief Define sync. mode for ADC sampling. Default 1 is 'ADC synchronised to PWM'
 */
#define _LOCK_ADC_TO_PWM 1

/**
 * @brief Define type for Port timer values. See also PORT_TIME_MASK
 */
typedef unsigned short PORT_TIME_TYP;

#endif /* _APP_GLOBAL_H_ */
