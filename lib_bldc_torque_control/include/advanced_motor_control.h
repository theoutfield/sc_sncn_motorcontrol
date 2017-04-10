/*
 * advanced_motor_control.h
 *
 *  Created on: March 23 2017
 *      Author: Synapticon
 */

#ifndef ADVANCED_MOTOR_CONTROL_H_
#define ADVANCED_MOTOR_CONTROL_H_

/*****************************************************************************/

/**
 * @brief Service to control the output torque of a 3-phase BLDC motor.
 * it also provides the measured data. The measured data includes:
 *      - ADC outputs including
 *      - position sensor outputs including
 *      - system error status
 *
 * @param motorcontrol_config   Structure for Motorcontrol Service configuration
 * @param i_adc                 Interface to communicate with the ADC server, and receive the ADC measurements
 * @param i_shared_memory       Interface to communicate with shared_memory_service and receive the position-related information
 * @param i_watchdog            Interface to communicate with watchdog_service
 * @param i_torque_control[2]     Array of interfaces to communicate with up to two clients for motor_control_service.
 * @param i_update_pwm          Interface to communicate with PWM module
 * @param ifm_tile_usec         Reference clock frequency of IFM tile (in MHz)
 *
 * @return void
 */

void torque_control_service( MotorcontrolConfig &motorcontrol_config,
                            interface ADCInterface client ?i_adc,
                            client interface shared_memory_interface ?i_shared_memory,
                            interface WatchdogInterface client i_watchdog,
                            interface TorqueControlInterface server i_torque_control[2],
                            client interface UpdatePWM i_update_pwm,
                            int ifm_tile_usec);

#endif /* ADVANCED_MOTOR_CONTROL_H_ */
