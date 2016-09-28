/*
 * advanced_motor_control.h
 *
 *  Created on: Aug 2, 2016
 *      Author: ramin
 */


#ifndef ADVANCED_MOTOR_CONTROL_H_
#define ADVANCED_MOTOR_CONTROL_H_

/*****************************************************************************/
void Motor_Control_Service( MotorcontrolConfig &motorcontrol_config,
                            interface ADCInterface client ?i_adc,
                            client interface shared_memory_interface ?i_shared_memory,
                            interface WatchdogInterface client i_watchdog,
                            interface MotorcontrolInterface server i_motorcontrol[4],
                            client interface update_pwm i_update_pwm);


#endif /* ADVANCED_MOTOR_CONTROL_H_ */
