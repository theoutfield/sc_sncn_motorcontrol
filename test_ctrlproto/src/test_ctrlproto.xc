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
#include <ioports.h>
#include <refclk.h>
#include <pwm_config.h>
#include <pwm_service_inv.h>
#include <sine_table_big.h>
#include <comm_loop.h>
#include <hall_server.h>
#include <hall_client.h>
#include <qei_server.h>
#include <qei_client.h>
#include <adc_ad7949.h>
#include <adc_client_ad7949.h>
#include <flash_somanet.h>
#include <ctrlproto.h>

#include <drive_config.h>
#include <drive_ctrl.h>
#include <internal_config.h>
#include <dc_motor_config.h>

#include <xscope.h>
#include <print.h>
#include <test.h>

int checkout = 0;

#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;

#define HALL 1
#define QEI 2

ctrl_proto_values_t InOut;

int get_statusword(chanend info)
{
	int statusword;
	info <: 1;
	info :> statusword;
	return statusword;
}

void set_controlword(int controlword, chanend info)
{
	info <: 2;
	info <: controlword;
}

void run_drive(chanend info) {
	int ready = 0;
	int switch_enable = 0;
	int status_word = 0;
	int switch_on_state = 0;
	int op_enable_state = 0;
	int control_word;

	while(!ready)
	{
		//check ready
		status_word = get_statusword(info);
		ready = check_ready(status_word);
	}
#ifndef print_slave
	printstrln("ready");
#endif

	while(!switch_enable)
	{
		//check switch
		status_word = get_statusword(info);
		switch_enable = check_switch_enable(status_word);
	}
#ifndef print_slave
	printstrln("switch_enable");
#endif

	//out CW for S ON
	control_word = SWITCH_ON_CONTROL;
	set_controlword(control_word, info);


	while(!switch_on_state)
	{
		set_controlword(control_word, info);
		printintln(control_word);
		//check switch_on_state
		status_word = get_statusword(info);
		switch_on_state = check_switch_on(status_word);
	}

#ifndef print_slave
	printstrln("switch_on_state");
#endif
	//out CW for En OP
	control_word = ENABLE_OPERATION_CONTROL;
	set_controlword(control_word, info);

	while(!op_enable_state)
	{
		//check op_enable_state
		status_word = get_statusword(info);
		op_enable_state = check_op_enable(status_word);
	}

#ifndef print_slave
	printstrln("op_enable_state");
#endif




}

int read_fault()
{
	return 0;
}

//int read_check()
//{
//	return check_out;
//}
int main(void)
{
	chan c_adc, c_adctrig;
	chan c_qei;
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4;
	chan c_pwm_ctrl, c_commutation;
	chan dummy, dummy1, dummy2;
	chan signal_adc;
	chan sig_1, c_signal;
	chan c_velocity_ctrl, c_position_ctrl;

	//etherCat Comm channels
	chan coe_in; 	///< CAN from module_ethercat to consumer
	chan coe_out; 	///< CAN from consumer to module_ethercat
	chan eoe_in; 	///< Ethernet from module_ethercat to consumer
	chan eoe_out; 	///< Ethernet from consumer to module_ethercat
	chan eoe_sig;
	chan foe_in; 	///< File from module_ethercat to consumer
	chan foe_out; 	///< File from consumer to module_ethercat
	chan pdo_in;
	chan pdo_out;

	chan info;
	//
	par
	{
		on stdcore[0] :
		{
			ecat_init();

			ecat_handler(coe_out, coe_in, eoe_out, eoe_in, eoe_sig, foe_out, foe_in, pdo_out, pdo_in);
		}

		on stdcore[0] :
		{
			firmware_update(foe_out, foe_in, sig_1); // firmware update
		}

		on stdcore[1]:
		{
			xscope_register(2, XSCOPE_CONTINUOUS, "0 actual_position", XSCOPE_INT,	"n",
							    XSCOPE_CONTINUOUS, "1 target_position", XSCOPE_INT, "n");

			xscope_config_io(XSCOPE_IO_BASIC);
		}

		on stdcore[1]:
		{
			run_drive(info);
		}

		on stdcore[2]:
		{
			int state;
			int check;
			int ctrl_input;
			int fault;

			int statusword;
			int controlword;
			int cmd;

			state = init_state(); //init state

			while(1)
			{
				select
				{
					case info :> cmd:
						if (cmd == 1)
							info <: statusword;
						else if (cmd == 2)
							info :> controlword;
					break;
				}

				switch(state)
				{
					case 1:
						check = 1;//read_check();
						fault = read_fault();
						state = get_next_values(state, check, 0, fault);
#ifdef print_slave
						printstr("updated state ");
						printhexln(state);
#endif
						break;

					case 2:
						check = 1;//read_check();
						fault = read_fault();
						state = get_next_values(state, check, 0, fault);
#ifdef print_slave
						printstr("updated state ");
						printhexln(state);
#endif
						break;

					case 7:
						check = 1;//read_check();
						//printintln(controlword);
						ctrl_input = read_controlword_switch_on(controlword);
						fault = read_fault();
						state = get_next_values(state, check, ctrl_input, fault);
#ifdef print_slave
						printstr("updated state ");
						printhexln(state);
#endif
						break;

					case 3:
						check = 1;//read_check();
						ctrl_input = read_controlword_enable_op(controlword);
						fault = read_fault();
						state = get_next_values(state, check, ctrl_input, fault);
#ifdef print_slave
						printstr("updated state ");
						printhexln(state);
#endif
						break;

					case 4:
						check = 1;//read_check();
						ctrl_input = read_controlword_quick_stop(controlword); /*quick stop*/
						fault = read_fault();
						state = get_next_values(state, check, ctrl_input, fault);
#ifdef print_slave
						printstr("updated state ");
						printhexln(state);
#endif
						break;

					case 5:
						check = 1;//read_check();
						ctrl_input = read_controlword_fault_reset(controlword);
						fault = read_fault();
						state = get_next_values(state, check, ctrl_input, fault);
#ifdef print_slave
						printstr("updated state ");
						printhexln(state);
#endif
						break;

					default:
						break;
				}
				statusword = update_statusword(statusword, state);
			}
		}


		/************************************************************
		 * IFM_CORE
		 ************************************************************/
		on stdcore[IFM_CORE]:
		{
			par
			{

		/*		adc_ad7949_triggered(c_adc, c_adctrig, clk_adc,
						p_ifm_adc_sclk_conv_mosib_mosia, p_ifm_adc_misoa,
						p_ifm_adc_misob);

				do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port,
						p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);

				commutation_sinusoidal(c_commutation, c_hall_p1, c_pwm_ctrl, signal_adc, c_signal); // hall based sinusoidal commutation


				run_hall( p_ifm_hall, c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4); // channel priority 1,2..4

				run_qei(c_qei, p_ifm_encoder);
		 */
			}
		}

	}

	return 0;
}
