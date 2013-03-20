/**************************************************************************
 * \file main.xc
 *	Motor Control main file
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Authors:  Pavan Kanajar <pkanajar@synapticon.com>, Ludwig Orgler <orgler@tin.it>
 * 			 & Martin Schwarz <mschwarz@synapticon.com>
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **************************************************************************/
#define Quadrature_Encoder_used

#include <xs1.h>
#include <platform.h>
#include <stdio.h>
#include <stdint.h>
#include <ioports.h>
#include <refclk.h>
#include <print.h>
#include <flash_Somanet.h>
#include <ethercat_handlers.h>
#include <ctrlproto.h>
#include <dc_motor_config.h>
#include <pwm_service_inv.h>
#include <comm_loop.h>
#include <adc_ad7949.h>
#include <hall_input.h>
#include <torque_ctrl.h>
#include "set_cmd.h"
#include<xscope.h>
#ifdef Quadrature_Encoder_used
	#include "qei_server.h"
	#include "qei_client.h"
#endif

#define COM_CORE 0
#define IFM_CORE 3
on stdcore[IFM_CORE]: clock clk_adc  = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm  = XS1_CLKBLK_REF;

torq_par t_params;
field_par f_params;
loop_par loop;
hall_par h_pole;
qei_par q_max;

int main(void)
{

	/*********************** EtherCAT Comm channels ***********************/
	chan coe_in, coe_out;   			///< CAN Channels
	chan eoe_in, eoe_out, eoe_sig;  	///< Ethernet Channels
	chan foe_in, foe_out;   			///< Firmware handler Channels
	chan pdo_in, pdo_out;  				///< EtherCAT comm. Channels
	chan sig_1, sig_3, sig_4;							///< auto reset channel

	/***********************Motor Control channels*************************/
	chan c_adc, c_adctrig;				///< ADC Channels
	chan c_hall;						///< Hall position Channels
	chan c_pwm_ctrl, c_value; 			///< Commutation Channnels
	chan input_torque, sig;				///< Torque Control Channels
	chan sensor_output;					///< Sensor out to EtherCAT
	streaming chan c_qei;



	/**********************************************************************
	 * CORE 0            	Ethercat communication
	 **********************************************************************/
	par{

			on stdcore[0] : {
				ecat_init();
				ecat_handler(coe_out, coe_in, eoe_out, eoe_in, eoe_sig, foe_out, foe_in, pdo_out, pdo_in);
			}

			on stdcore[0] : {
				check_file(foe_out, foe_in, sig_1); 						// firmware update thread
			}
/*
		 on stdcore[1]: {

				 xscope_register(9,
						 XSCOPE_CONTINUOUS, "0 iId", XSCOPE_INT, "n",
					 XSCOPE_CONTINUOUS, "1 iIq", XSCOPE_INT, "n",
					 XSCOPE_CONTINUOUS, "2 Vdout", XSCOPE_INT, "n",
					 XSCOPE_CONTINUOUS, "3 Vqout", XSCOPE_INT, "n",
					 XSCOPE_CONTINUOUS, "4 ialpha", XSCOPE_UINT, "n",
					 XSCOPE_CONTINUOUS, "5 ibeta", XSCOPE_UINT, "n",
					 XSCOPE_CONTINUOUS, "6 iVectorCurrent", XSCOPE_UINT, "n",
					 XSCOPE_CONTINUOUS, "7 iIdPeriod2", XSCOPE_UINT, "n",
					 XSCOPE_CONTINUOUS, "8 iIqPeriod2", XSCOPE_UINT, "n"
				);


		 }*/

	/**********************************************************************
	 * CORE 2          Communication with the Motor Control
	 **********************************************************************/
		on stdcore[2]:par
		{
			{
				int torque_set;
				#ifdef Quadrature_Encoder_used
					unsigned position_read = 0, speed_read = 0, v=0;
				#else
					int position_read = 0, speed_read = 0;
				#endif
				ctrl_proto_values_t InOut;
				timer tx; unsigned  ts; tx :> ts;
				tx when timerafter(ts+4*SEC_FAST) :> void;
#ifdef Quadrature_Encoder_used
				init_qei(q_max);
#endif
				init_ctrl_proto(InOut);
				while(1)
				{
					ctrlproto_protocol_handler_function(pdo_out,pdo_in,InOut);
					torque_set = InOut.in_torque;
					input_torque <: 20;         	  								//torque control token
					input_torque <: torque_set;										//torque input

					#ifdef Quadrature_Encoder_used
						{speed_read, position_read, v} = get_qei_data(c_qei, q_max);
						InOut.out_position = position_read;								//quadrature position output
						InOut.out_speed    = speed_read;								//quadrature speed output
					#else
						speed_read = get_hall_speed(sensor_output);						//hall speed output
						position_read = get_hall_angle(sensor_output);					//hall position output
						InOut.out_position = position_read;								//quadrature position output
												InOut.out_speed    = speed_read;								//quadrature speed output
						//xscope_probe_data(0,position_read);
					#endif
				}
			}

			/*{
				int torque_set=-190; unsigned position_read = 0, speed_read = 0, v=0;
				ctrl_proto_values_t InOut;
				timer tx; unsigned  ts; tx :> ts;
				tx when timerafter(ts+4*SEC_FAST) :> void;							// time delay for the initialization
			//	init_qei(q_max);													// initialize max quadrature encoder count
				init_ctrl_proto(InOut);
				input_torque <: 20;         	  								//torque control token
									input_torque <:-190;										//torque input
				while(1)
				{
					//ctrlproto_protocol_handler_function(pdo_out,pdo_in,InOut);      // read the incoming etherCAT messages
					//torque_set = InOut.in_torque;
					input_torque <: 20;         	  								//torque control token
					input_torque <: torque_set;										//torque input

				//	{speed_read, position_read, v} = get_qei_data(c_qei, q_max);    ///QEI client to get the values from the QEI server
				//	#ifndef Quadrature_Encoder_used
						//sensor_output <: 1; 										//position read token
						//sensor_output :> position_read;								//hall position output
						speed_read = get_hall_speed(sensor_output);						//hall speed output
												position_read = get_hall_angle(sensor_output);					//hall position output
				//	#else

					// Output the sensor information back to EtherCAT //
					InOut.out_position = position_read;								//quadrature position output
					xscope_probe_data(0,position_read);
					InOut.out_speed    = speed_read;								//quadrature speed output
					xscope_probe_data(1,speed_read);
					//#endif
				}
			}*/
			/*{
				tor_data send;
				timer tx;unsigned  ts;int valid=0;
				int torq;
				tx :> ts;  // first value
				tx when timerafter(ts+4*SEC_FAST) :> ts;
				while(1)
				{
					valid = input_tor_cmd(send );
					printintln( send.var1);
					torq = send.var1;
					input_torque <: 20;
					input_torque <: torq;
				}
			}*/

		}


		/******************************************************************
		 * CORE 3          			Motor Control
		 ******************************************************************/
		on stdcore[IFM_CORE]: {
			par {
					adc_ad7949_triggered( c_adc, c_adctrig, clk_adc, p_ifm_adc_sclk_conv_mosib_mosia, p_ifm_adc_misoa, p_ifm_adc_misob);

					do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port, p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);

					{
						init_hall(h_pole);
						run_hall(c_hall, sensor_output, p_ifm_hall, h_pole);
					}

					commutation(c_value, c_pwm_ctrl, sig);

					{
						init_params_struct_all(t_params, f_params, loop);
						init_loop_pars(loop);
						init_torque_pars(t_params);
						foc_loop(sig, input_torque, c_adc, c_hall, c_value);
					}
					#ifdef Quadrature_Encoder_used
					{
						do_qei( c_qei, p_ifm_encoder );
					}
					#endif
			  }
		}

	}

	return 0;

}


