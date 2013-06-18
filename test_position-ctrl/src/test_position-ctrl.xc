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


#define SET_POSITION_TOKEN 40

//basic position ctrl test
void set_position_test(chanend c_position_ctrl)
{
	int position = 0;
	in_data d;
	timer ts;
	unsigned time;
	int increment = 50;

	ts:>time;

	ts when timerafter(time+10*SEC_STD) :> time;

	while (1) {
		//input_pos(d);
		//printintln(d.set_position);

		ts when timerafter(time+100000) :> time;

		position +=increment;
		c_position_ctrl <: 2;
		c_position_ctrl <: position;
		if(position>300000)
			increment *= -1;
		if(position<0)
			increment *= -1;

	}
}


//internal
void set_position(int target_position, chanend c_position_ctrl) {
	c_position_ctrl <: SET_POSITION_TOKEN;
	c_position_ctrl <: target_position;
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

//csv mode function
void set_position_csv(csp_par &csp_params, int target_position,
		int position_offset, int velocity_offset, int torque_offset, chanend c_position_ctrl)
{
	set_position( position_limit(	(target_position + position_offset) * csp_params.base.polarity, csp_params.max_position_limit, csp_params.min_position_limit  ), c_position_ctrl );
}

void position_control(chanend c_hall, chanend c_signal, chanend c_commutation, chanend c_position_ctrl)
{
	int actual_position = 0;
	timer ts;
	unsigned int time;
	int error_position = 0;
	int error_position_D = 0;
	int error_position_I = 0;
	int previous_error = 0;
	int position_control_out = 0;
	int Kp = 6, Kd = 0, Ki = 0;
	int max_integral = (13739)/1;
	int target_position = 15000;
	int command;

	//check init signal from commutation level
	while (1) {
		unsigned received_command = 0;
		select
		{
			case c_signal :> command:
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
	ts when timerafter(time+3*SEC_FAST) :> time;
	//set_commutation_sinusoidal(c_commutation, 1000);
	while(1)
	{
		select{
			case c_position_ctrl :> command:
				if(command == SET_POSITION_TOKEN)
				{
					c_position_ctrl :> target_position;
				}
				break;
			default:
				break;
		}

		ts when timerafter(time+100000) :> time; //1khz

		actual_position = get_hall_absolute_pos(c_hall);

		//xscope_probe_data(0, actual_position);

		error_position = (target_position - actual_position)*1000;
		error_position_I = error_position_I + error_position;
		error_position_D = error_position - previous_error;

		if(error_position_I > max_integral*1000)
			error_position_I = max_integral*1000;
		else if(error_position_I < -max_integral*1000)
			error_position_I = 0 - max_integral*1000;

		position_control_out = (Kp*error_position)/10000 + (Ki*error_position_I) + (Kd*error_position_D);

		if(position_control_out > 13739)
			position_control_out = 13739;
		else if(position_control_out < -13739)
			position_control_out = 0-13739;


		set_commutation_sinusoidal(c_commutation, position_control_out);

		#ifdef ENABLE_xscope_main
		xscope_probe_data(0, actual_position);
		xscope_probe_data(1, target_position);
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
par		{

			set_position_test(c_position_ctrl);

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
			position_control( c_hall_p2, c_signal, c_commutation, c_position_ctrl);
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
