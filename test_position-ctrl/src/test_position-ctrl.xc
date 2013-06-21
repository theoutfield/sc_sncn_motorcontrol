/*
 *
 * \file
 * \brief Main project file
 *
 * Port declarations, etc.
 *
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 0.1 (2012-11-23 1850)
 *\Motor 3 motion profile size optimized code for position ctrl loops
 */

#include <xs1.h>
#include <platform.h>
#include <print.h>
#include <stdio.h>
#include <stdint.h>
#include "ioports.h"
#include "hall_server.h"
#include "hall_client.h"
#include "qei_client.h"
#include "pwm_service_inv.h"
#include "adc_ad7949.h"
#include "test.h"
#include "pwm_config.h"
#include "comm_loop.h"
#include "refclk.h"
#include "velocity_ctrl.h"
#include <xscope.h>
#include "qei_client.h"
#include "qei_server.h"
#include "adc_client_ad7949.h"
#include <dc_motor_config.h>
#include "sine_table_big.h"
#include "print.h"
#include "filter_blocks.h"
#include "profile.h"

#include <flash_Somanet.h>
#include <internal_config.h>
#include <ctrlproto.h>

#define ENABLE_xscope_main
#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;

extern int init(int vel, int pos_i, int pos_f);
extern int mot_q(int i);
extern int position_factor(int gear_ratio, int qei_max_real, int pole_pairs, int sensor_used);

#define SET_POSITION_TOKEN 40
#define HALL 1
#define QEI 2

#define HALL_PRECISION		2
#define QEI_PRECISION		512

//internal
void set_position(int target_position, chanend c_position_ctrl) {
	c_position_ctrl <: SET_POSITION_TOKEN;	//	POSITION_CTRL_WRITE(SET_POSITION_TOKEN);
	c_position_ctrl <: target_position;  	//	POSITION_CTRL_WRITE(target_position);
}

int position_limit(int position, int max_position_limit, int min_position_limit) {
	if (position > max_position_limit)
	{
		return max_position_limit;
	}
	else if (position < min_position_limit)
	{
		return min_position_limit;
	}
	else if (position >= min_position_limit && position <= max_position_limit)
	{
		return position;
	}
}

//csp mode function
void set_position_csp(csp_par &csp_params, int target_position,
		int position_offset, int velocity_offset, int torque_offset, chanend c_position_ctrl)
{
	set_position(position_limit( (target_position + position_offset) * csp_params.base.polarity , csp_params.max_position_limit*1000, csp_params.min_position_limit*1000), c_position_ctrl);
}



//basic position ctrl test
void set_position_test(chanend c_position_ctrl)
{
	int position = 0;
	in_data d;
	timer ts;
	unsigned time;
	int increment = 50;

	int pos_m;
	csp_par csp_params;

	init_csp(csp_params);


	//ts when timerafter(time+10*SEC_STD) :> time;
	//check init
	while (1) {
		unsigned command, received_command = 0;
		select
		{
			case c_position_ctrl :> command: 			//SIGNAL_READ(command):
				received_command = 1;
			break;
			default:
			break;
		}
		if(received_command == 1)
		{
			printstrln(" init posctrl ");
			break;
		}
	}

	ts:>time;
	while (1) {
		//input_pos(d);
		//printintln(d.set_position);

		ts when timerafter(time+100000) :> time;

		position +=increment;
		//set_position(position, c_position_ctrl);
		//xscope_probe_data(0, position);
		xscope_probe_data(1, position);
		set_position_csp(csp_params, position, 0, 0, 0, c_position_ctrl);

		//xscope_probe_data(1, pos_m);
		if(position>300000)
			increment *= -1;
		if(position<0)
			increment *= -1;

	}
}
void profile_pos(chanend c_position_ctrl)
{
	int samp;
	int v = 440, i =1;
	int cur_p = 0, d_pos = 270;
	int p_ramp;
	timer ts;
	unsigned time;
	csp_par csp_params;
	init_csp(csp_params);

	//check init
	while (1) {
		unsigned command, received_command = 0;
		select
		{
			case c_position_ctrl :> command: 			//SIGNAL_READ(command):
				received_command = 1;
			break;
			default:
			break;
		}
		if(received_command == 1)
		{
			printstrln(" init posctrl ");
			break;
		}
	}

	samp = init(v, cur_p, d_pos);

	ts:>time;
	while(i<samp)
	{
		ts when timerafter(time+100000) :> time;
		p_ramp = mot_q(i);
		i++;
		xscope_probe_data(1, p_ramp);
		set_position_csp(csp_params, p_ramp, 0, 0, 0, c_position_ctrl);
	}
	while(1)
	{
		ts when timerafter(time+100000) :> time;
		xscope_probe_data(1, p_ramp);
		set_position_csp(csp_params, p_ramp, 0, 0, 0, c_position_ctrl);
	}
}



void position_control(hall_par hall_params, qei_par qei_params, chanend c_hall, chanend c_qei, chanend c_signal, chanend c_commutation, chanend c_position_ctrl, int sensor_used)
{
	int actual_position = 0;
	int target_position = 0;

	int error_position = 0;
	int error_position_D = 0;
	int error_position_I = 0;
	int previous_error = 0;
	int position_control_out = 0;

	int Kp = 10, Kd = 0, Ki = 0;
	int max_integral = (13739*102000)/1;

	timer ts;
	unsigned int time;

	int command;
	int direction = 0;

	int precision;
	int precision_factor;

	//check init signal from commutation level
	while (1)
	{
		unsigned received_command = 0;
		select
		{
			case c_signal :> command: 			//SIGNAL_READ(command):
				received_command = 1;
				break;
			default:
				break;
		}
		if(received_command == 1)
		{
			printstrln(" init commutation");
			break;
		}
	}

	ts:> time;
	ts when timerafter(time+SEC_STD) :> time;

	c_position_ctrl <: 1; //start


	if(sensor_used == HALL)
	{
		precision_factor = position_factor(hall_params.gear_ratio, 1, hall_params.pole_pairs, sensor_used);
		precision = HALL_PRECISION;
	}
	else if(sensor_used == QEI)
	{
		precision_factor = position_factor(qei_params.gear_ratio, qei_params.real_counts, 1, sensor_used);
		precision = QEI_PRECISION;
	}

	//set_commutation_sinusoidal(c_commutation, 1000);
	while(1)
	{
		select{
			case c_position_ctrl :> command:			//  POSITION_CTRL_READ(command);
				if(command == SET_POSITION_TOKEN)
				{
					c_position_ctrl :> target_position;  //	POSITION_CTRL_READ(target_position);

				}
				break;
			default:
				break;
		}

		ts when timerafter(time + MSEC_STD) :> time; //1khz

		if(sensor_used == HALL)
		{
			actual_position = ( ( ( (get_hall_absolute_pos(c_hall)/500) * precision_factor)/precision )/819 )*100;   // 100/(500*819) ~ 1/4095 appr (hall)
		}
		else if(sensor_used == QEI)
		{
			{actual_position, direction} =  get_qei_position_count(c_qei);
			actual_position = (actual_position * precision_factor)/precision;
		}
		//xscope_probe_data(0, actual_position);

		error_position = (target_position - actual_position);
		error_position_I = error_position_I + error_position;
		error_position_D = error_position - previous_error;

		if(error_position_I > max_integral)
			error_position_I = max_integral;
		else if(error_position_I < -max_integral)
			error_position_I = 0 - max_integral;

		position_control_out = (Kp*error_position)/2000 + (Ki*error_position_I)/102000 + (Kd*error_position_D)/10000;

		if(position_control_out > 13739)
			position_control_out = 13739;
		else if(position_control_out < -13739)
			position_control_out = 0-13739;


		set_commutation_sinusoidal(c_commutation, position_control_out);

		#ifdef ENABLE_xscope_main
		xscope_probe_data(0, actual_position);
		//xscope_probe_data(1, target_position);
		#endif

		previous_error = error_position;

	}
}

int main(void) {
	chan c_adc, c_adctrig;
	chan c_qei;
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4;
	chan c_pwm_ctrl, c_commutation;
	chan dummy, dummy1, dummy2;
	chan speed_out, stop, str, info;
	chan enco_1, sync_output;
	chan signal_adc, c_value, input;
	chan c_torque;
	chan sig_1, c_signal;
	chan c_velocity_ctrl, c_position_ctrl;
	//etherCat Comm channels
	chan coe_in; ///< CAN from module_ethercat to consumer
	chan coe_out; ///< CAN from consumer to module_ethercat
	chan eoe_in; ///< Ethernet from module_ethercat to consumer
	chan eoe_out; ///< Ethernet from consumer to module_ethercat
	chan eoe_sig;
	chan foe_in; ///< File from module_ethercat to consumer
	chan foe_out; ///< File from consumer to module_ethercat
	chan pdo_in;
	chan pdo_out;

	//
	par
	{
/*		on stdcore[0] :
		{
			ecat_init();
			ecat_handler(coe_out, coe_in, eoe_out, eoe_in, eoe_sig, foe_out,
					foe_in, pdo_out, pdo_in);
		}

		on stdcore[0] :
		{
			ether_comm(pdo_out, pdo_in, c_signal, c_velocity_ctrl);
		}*/
		on stdcore[1]:
		{
			par
			{

				//set_position_test(c_position_ctrl);
				profile_pos(c_position_ctrl);
				/*{
					int gear = 2634, poles = 8, qei_max = 4000;
					int factor;
					factor = position_factor(gear, qei_max, poles, HALL);
					printintln(factor);

				}*/
			}
		}

		on stdcore[1]:
		{
			xscope_register(14, XSCOPE_CONTINUOUS, "0 hall(delta)", XSCOPE_INT,
					"n", XSCOPE_CONTINUOUS, "1 actualspeed", XSCOPE_INT, "n",
					XSCOPE_CONTINUOUS, "2 ramp", XSCOPE_INT, "n",
					XSCOPE_CONTINUOUS, "3 ep", XSCOPE_INT, "n", XSCOPE_DISCRETE,
					"4 ev", XSCOPE_INT, "n", XSCOPE_CONTINUOUS, "5 pos_d",
					XSCOPE_INT, "n", XSCOPE_CONTINUOUS, "6 vel_d", XSCOPE_INT,
					"n", XSCOPE_CONTINUOUS, "7 speed", XSCOPE_INT, "n",
					XSCOPE_CONTINUOUS, "8 sinepos_a", XSCOPE_UINT, "n",
					XSCOPE_CONTINUOUS, "9 sinepos_b", XSCOPE_UINT, "n",
					XSCOPE_CONTINUOUS, "10 sinepos_c", XSCOPE_UINT, "n",
					XSCOPE_CONTINUOUS, "11 sine_a", XSCOPE_UINT, "n",
					XSCOPE_CONTINUOUS, "12 sine_b", XSCOPE_UINT, "n",
					XSCOPE_CONTINUOUS, "13 sine_c", XSCOPE_UINT, "n");
			xscope_config_io(XSCOPE_IO_BASIC);
		}

		on stdcore[2]:
		{
			par
			{
				{
					 //ctrl_par position_ctrl_params;
					 hall_par hall_params;
					 qei_par qei_params;

					 //init_position_control(position_ctrl_params);

					 init_hall(hall_params);
					 init_qei(qei_params);

					 position_control(hall_params, qei_params, c_hall_p2, c_qei, c_signal, c_commutation, c_position_ctrl, QEI);
				}

			}

		}

		/************************************************************
		 * IFM_CORE
		 ************************************************************/
		on stdcore[IFM_CORE]:
		{
			par
			{

				adc_ad7949_triggered(c_adc, c_adctrig, clk_adc,
						p_ifm_adc_sclk_conv_mosib_mosia, p_ifm_adc_misoa,
						p_ifm_adc_misob);

				do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port,
						p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);

				commutation_sinusoidal(c_commutation, c_hall_p1, c_pwm_ctrl, signal_adc, c_signal); // hall based sinusoidal commutation


				run_hall( p_ifm_hall, c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4); // channel priority 1,2..4

				run_qei(c_qei, p_ifm_encoder);

			}
		}

	}

	return 0;
}
