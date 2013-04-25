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
#include "qei_server.h"
#include "qei_client.h"
#include "hall_input.h"
#include "hall_client.h"
#include "pwm_service_inv.h"
#include"pos_ctrl.h"
#include "pwm_config.h"
#include "comm_sine.h"
#include "refclk.h"
#include <xscope.h>
#include "adc_ad7949.h"
#include <dc_motor_config.h>
#define COM_CORE 0
#define IFM_CORE 3


hall_par h_pole;
//on stdcore[3]: in           port    ADC_SYNC_PORT = XS1_PORT_16A;

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;

int main(void)
{
	chan c_adc, adc_d, dirn;
	chan c_adctrig;
	chan c_qei;
	chan c_hall, c_hall1;
	chan c_pwm_ctrl;
	chan c_commutation;
	chan speed_read;
	chan enco, dummy;
	chan pos_ctrl;
	chan pos_data;
	chan speed_out, stop, str, info, r_hall;
	chan enco_1;

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
	chan sig_1;
	//
	par
	{

		//etherCat comm
		/*on stdcore[0] : {
		 ecat_init();
		 ecat_handler(coe_out, coe_in, eoe_out, eoe_in, eoe_sig, foe_out, foe_in, pdo_out, pdo_in);
		 }
		 on stdcore[0] : {
		 check_file(foe_out, foe_in, sig_1); // firmware update
		 }
		 */
		//
		on stdcore[1]:
		{
			xscope_register(14,
					XSCOPE_CONTINUOUS, "0 hall(delta)",	XSCOPE_INT, "n",
					XSCOPE_CONTINUOUS, "1 qei", XSCOPE_INT,"n",
					XSCOPE_CONTINUOUS, "2 pos", XSCOPE_INT, "n",
					XSCOPE_DISCRETE, "3 ep", XSCOPE_INT, "n",
					XSCOPE_DISCRETE,"4 ev", XSCOPE_INT, "n",
					XSCOPE_CONTINUOUS, "5 pos_d",XSCOPE_INT, "n",
					XSCOPE_CONTINUOUS, "6 vel_d", XSCOPE_INT,"n",
					XSCOPE_CONTINUOUS, "7 speed", XSCOPE_INT, "n",
					XSCOPE_CONTINUOUS, "8 sinepos_a", XSCOPE_UINT, "n",
					XSCOPE_CONTINUOUS, "9 sinepos_b", XSCOPE_UINT, "n",
					XSCOPE_CONTINUOUS, "10 sinepos_c", XSCOPE_UINT, "n",
					XSCOPE_CONTINUOUS, "11 sine_a", XSCOPE_UINT, "n",
					XSCOPE_CONTINUOUS, "12 sine_b", XSCOPE_UINT, "n",
					XSCOPE_CONTINUOUS, "13 sine_c", XSCOPE_UINT, "n");
			xscope_config_io(XSCOPE_IO_BASIC);
		}
		/*on stdcore[2]:
		{



			{
				int hall_pos, qei_pos; // input
				int sync_pos;          // output

				int speed, v;

				int max = 4095;
				unsigned int t1, t2;
				timer t_qei, t_hall;

				signed pos, spos, prev = 0, count = 0, co12 = 0, first = 1,	set = 0;
				int max_count = GEAR_RATIO * QEI_COUNTS;
				int enc_div = POLE_PAIRS * GEAR_RATIO;
				int hall_p;
				int diffi, count1;

				t_qei:> t1; t_hall :> t2;
				t_qei when timerafter(t1+ 7*SEC_STD) :> t1;
				while(1)
				{
					select
					{
						case t_qei  when timerafter(t1 + 700) :> t1:
								{qei_pos, v} = get_qei_pos(c_qei); //acq     //{speed, qei_pos, v} = get_qei_data(c_qei);
								qei_pos = max - qei_pos;
								xscope_probe_data(0, qei_pos);
							break;

						case t_hall when timerafter(t2 + 7000) :> t2: //4khz  20000
								//hall_pos = get_hall_angle(c_hall1);
								//xscope_probe_data(0, hall_pos);
							break;
					}
					//t when timerafter(time+ 10*USEC_STD) :> time; // acq time


					// add a counter for qei acq



				}
			}

		}

*/
		/************************************************************
		 * IFM_CORE
		 ************************************************************/
		on stdcore[IFM_CORE]: {
			par {
				do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port, p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);

				//adc_ad7949( c_adc, clk_adc, p_ifm_adc_sclk_conv_mosib_mosia, p_ifm_adc_misoa, p_ifm_adc_misob);

				commutation_test(c_commutation, c_hall, c_pwm_ctrl, c_adc);

				//run_hall( c_hall, p_ifm_hall, c_hall1, r_hall);


				{
					init_hall(h_pole);
					run_hall(c_hall, p_ifm_hall, h_pole);
				}
				//do_qei( c_qei, p_ifm_encoder );

				{

					unsigned time = 0;
					int speed;
					timer t;
					t:>time;
					t when timerafter(time+700*USEC_FAST) :> time;
					c_commutation <: 2;
					c_commutation <: -889;

					while(1)
					{
						// c_commutation <: 1;
						// c_commutation :> speed;
						// xscope_probe_data(1,speed);
					}
				}


			}
		}

	}

	return 0;
}



