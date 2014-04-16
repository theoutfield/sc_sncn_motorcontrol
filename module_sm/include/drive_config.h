
/**
 * \file drive_config.h
 * \brief Motor Drive defines and configurations
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */
/*
 * Copyright (c) 2014, Synapticon GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Execution of this software or parts of it exclusively takes place on hardware
 *    produced by Synapticon GmbH.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Synapticon GmbH.
 *
 */

#ifndef DRIVE_CONFIG_H_
#define DRIVE_CONFIG_H_
#pragma once

/**
 * \brief Modes of Operation (CiA402 Standard)
 *
 * 	M - Mandatory, C - Conditional, R - Recommended, O - optional , FG - Function Group
 */

#define	PP		1 	/* Profile Position mode 									O*/
#define VL 		2	/* Velocity mode (frequency converter) 						O*/
#define PV 		3   /* Profile velocity mode 									O*/
#define TQ 		4   /* Torque profile mode 										O*/
#define HM 		6	/* Homing mode 												O*/
#define IP 		7	/* Interpolated position mode 								O*/
#define CSP 	8	/* Cyclic synchronous position mode 						C*/
#define CSV 	9	/* Cyclic synchronous velocity mode 						C*/
#define CST 	10	/* Cyclic synchronous torque mode 							C*/
#define CSTCA 	11	/* Cyclic synchronous torque mode with commutation angle 	O*/

/* Manufacturer specific mode -128...-1 optional */

/* Controlword */

//Common for all Modes of Operation (CiA402)

#define SHUTDOWN				0x0006
#define SWITCH_ON				0x000F
#define QUICK_STOP  			0x000B
#define CLEAR_FAULT 			0x0080

//Operation Mode specific control words (complies with CiA402)

/* Homing mode */
#define START_HOMING 			0x001F
#define HALT_HOMING  			0x011F

/* Profile Position Mode */
#define ABSOLUTE_POSITIONING 	0x001F
#define RELATIVE_POSITIONING 	0x005F   	// supported currently
#define STOP_POSITIONING		0x010F

/*Profile Velocity Mode*/
#define HALT_PROFILE_VELOCITY	0x010F

/* Statusword */
//state defined is ORed with current state

#define TARGET_REACHED 			0x0400

/* Homing Mode */
#define HOMING_ATTAINED			0x1000

/* Profile Position Mode */
#define SET_POSITION_ACK	  	0x1000

/* Profile Velocity Mode */
#define TARGET_VELOCITY_REACHED 0x0400

/*Controlword Bits*/
#define SWITCH_ON_CONTROL					0x1
#define ENABLE_VOLTAGE_CONTROL				0x2
#define QUICK_STOP_CONTROL					0x4
#define ENABLE_OPERATION_CONTROL			0x8
#define OPERATION_MODES_SPECIFIC_CONTROL	0x70  /*3 bits*/
#define FAULT_RESET_CONTROL					0x80
#define HALT_CONTROL						0x100
#define OPERATION_MODE_SPECIFIC_CONTROL		0x200
#define RESERVED_CONTROL					0x400
#define MANUFACTURER_SPECIFIC_CONTROL		0xf800

/*Statusword Bits*/
#define READY_TO_SWITCH_ON_STATE	  		0X1
#define SWITCHED_ON_STATE					0X2
#define OPERATION_ENABLED_STATE				0X4
#define FAULT_STATE							0X8
#define VOLTAGE_ENABLED_STATE				0X10
#define QUICK_STOP_STATE					0X20
#define SWITCH_ON_DISABLED_STATE			0X40
#define WARNING_STATE						0X80
#define MANUFACTURER_SPECIFIC_STATE			0X100
#define REMOTE_STATE						0X200
#define TARGET_REACHED_OR_RESERVED_STATE	0X400
#define INTERNAL_LIMIT_ACTIVE_STATE			0X800
#define OPERATION_MODE_SPECIFIC_STATE		0X1000	// 12 CSP/CSV/CST  13
#define MANUFACTURER_SPECIFIC_STATES   		0XC000	// 14-15

typedef int bool;
enum { false, true };

typedef struct S_Check_list
{
	bool ready;
	bool switch_on;
	bool operation_enable;
	bool mode_op;
	bool fault;

	bool _commutation_init;
	bool _hall_init;
	bool _qei_init;
	bool _adc_init;
	bool _torque_init;
	bool _velocity_init;
	bool _position_init;

}check_list;


bool __check_bdc_init(chanend c_signal);
/**
 * \brief Check commutation initialization
 *
 *  Output
 * \return init state of the commutation loop
 */
bool __check_commutation_init(chanend c_signal);

/**
 * \brief Check hall initialization
 *
 *  Output
 * \return init state of the hall loop
 */
bool __check_hall_init(chanend c_hall);

/**
 * \brief Check qei initialization
 *
 *  Output
 * \return init state of the qei loop
 */
bool __check_qei_init(chanend c_qei);

/**
 * \brief Check adc initialization
 *
 *  Output
 * \return init state of the adc loop
 */
bool __check_adc_init();

/**
 * \brief Check torque control initialization
 *
 *  Output
 * \return init state of the torque control loop
 */
bool __check_torque_init(chanend c_torque_ctrl);

/**
 * \brief Check velocity control initialization
 *
 *  Output
 * \return init state of the velocity control loop
 */
bool __check_velocity_init(chanend c_velocity_ctrl);

/**
 * \brief Check position control initialization
 *
 *  Output
 * \return init state of the position control loop
 */
bool __check_position_init(chanend c_position_ctrl);

int init_state(void);

/**
 * \brief Initialize checklist params
 *
 *  Output
 * \return check_list initialised checklist parameters
 */
check_list init_checklist(void);

/**
 * \brief Update Checklist
 *
 *  Input channel
 * \channel c_commutation for communicating with the commutation server
 * \channel c_hall for communicating with the hall server
 * \channel c_qei for communicating with the qei server
 * \channel c_adc for communicating with the adc server
 * \channel c_torque_ctrl for communicating with the torque control server
 * \channel c_velocity_ctrl for communicating with the velocity control server
 * \channel c_position_ctrl for communicating with the position control server
 *
 * 	Input
 * \param mode sets mode of operation
 *
 *  Output
 * \return check_list_param updated checklist parameters
 */
void update_checklist(check_list &check_list_param, int mode, chanend c_commutation, chanend c_hall, chanend c_qei,
		chanend c_adc, chanend c_torque_ctrl, chanend c_velocity_ctrl, chanend c_position_ctrl);

int update_statusword(int current_status, int state_reached, int ack, int q_active, int shutdown_ack);

int get_next_state(int in_state, check_list &checklist, int controlword);

int read_controlword_switch_on(int control_word);

int read_controlword_quick_stop(int control_word);

int read_controlword_enable_op(int control_word);

int read_controlword_fault_reset(int control_word);

#endif /* DRIVE_CONFIG_H_*/
