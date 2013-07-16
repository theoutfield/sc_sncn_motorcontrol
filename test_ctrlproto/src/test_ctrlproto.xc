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
#include <internal_config.h>
#include <dc_motor_config.h>

#include <xscope.h>
#include <print.h>
#include <test.h>



#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;

#define HALL 1
#define QEI 2

ctrl_proto_values_t InOut;

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

		on stdcore[2]:
		{
			int current_state = 0;
			int sw;   // =  update_statusword(current_state, 1);

			in_data d;
			while(1)
			{
				input_new_state(d);
				printstr("state ");
				printintln(d.set_state);
				sw = update_statusword(current_state, d.set_state);
				printstr("updated state ");
				printhexln(sw);
				current_state = sw;
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
