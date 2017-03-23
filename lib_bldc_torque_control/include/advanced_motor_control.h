/*
 * advanced_motor_control.h
 *
 *  Created on: Aug 2, 2016
 *      Author: ramin
 */


#ifndef ADVANCED_MOTOR_CONTROL_H_
#define ADVANCED_MOTOR_CONTROL_H_

/*****************************************************************************/
void motor_control_service( MotorcontrolConfig &motorcontrol_config,
                            interface ADCInterface client ?i_adc,
                            client interface shared_memory_interface ?i_shared_memory,
                            interface WatchdogInterface client i_watchdog,
                            interface MotorControlInterface server i_motorcontrol[2],
                            client interface update_pwm i_update_pwm,
                            int ifm_tile_usec);


#endif /* ADVANCED_MOTOR_CONTROL_H_ */
