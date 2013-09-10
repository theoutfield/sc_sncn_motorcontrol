#ifndef DRIVE_CONFIG_H_
#define DRIVE_CONFIG_H_
#pragma once

/**
 * \brief Modes of Operation (CiA402)
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
//100
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
#define ABSOLUTE_POSITIONING 	0x001F	 	// not supported yet
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


bool __check_commutation_init(chanend c_signal);
bool __check_hall_init(chanend c_hall);
bool __check_qei_init(chanend c_qei);
bool __check_adc_init();
bool __check_torque_init(chanend c_torque_ctrl);
bool __check_velocity_init(chanend c_velocity_ctrl);
bool __check_position_init(chanend c_position_ctrl);

int init_state(void);

check_list init_checklist(void);

void update_checklist(check_list &check_list_param, int mode, chanend c_commutation, chanend c_hall, chanend c_qei,
		chanend c_adc, chanend c_torque_ctrl, chanend c_velocity_ctrl, chanend c_position_ctrl);

int update_statusword(int current_status, int state_reached, int ack, int q_active, int shutdown_ack);

int get_next_state(int in_state, check_list &checklist, int controlword);

int read_controlword_switch_on(int control_word);

int read_controlword_quick_stop(int control_word);

int read_controlword_enable_op(int control_word);

int read_controlword_fault_reset(int control_word);


#endif /* DRIVE_CONFIG_H_*/
