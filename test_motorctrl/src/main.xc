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
#include <ethercat.h>
#include <ctrlproto.h>
#include <dc_motor_config.h>
#include <pwm_service_inv.h>
#include <comm_loop.h>
#include <adc_ad7949.h>
#include <hall_server.h>
#include <torque_ctrl.h>

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
loop_par loop_params;
hall_par h_pole;
qei_par q_max;

int main(void)
{

	/*********************** EtherCAT Comm channels ***********************/
	chan coe_in, coe_out;   			///< CAN Channels
	chan eoe_in, eoe_out, eoe_sig;  	///< Ethernet Channels
	chan foe_in, foe_out;   			///< Firmware handler Channels
	chan pdo_in, pdo_out;  				///< EtherCAT comm. Channels
	chan sig_1, sig_3, sig_4;			///< auto reset channel

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
				ecat_handler(coe_out, coe_in, eoe_out, eoe_in, eoe_sig, foe_out, foe_in, pdo_out, pdo_in);  // ethercat Communication handlers
			}

			on stdcore[0] : {
				check_file(foe_out, foe_in, sig_1); 														// firmware update over ethercat thread
			}

	/**********************************************************************
	 * CORE 2          Communication with the Motor Control
	 **********************************************************************/
		on stdcore[2]:par
		{
			{
				int torque_set=50;
				#ifdef Quadrature_Encoder_used
					unsigned position_read = 0, speed_read = 0, v=0;
				#else
					int position_read = 0, speed_read = 0;
				#endif
				ctrl_proto_values_t InOut;
				timer tx; unsigned  ts; tx :> ts;
				tx when timerafter(ts+4*SEC_FAST) :> void;								// time delay for the initialization
				#ifdef Quadrature_Encoder_used
				init_qei(q_max);														// initialize max quadrature encoder count
				#endif
				init_ctrl_proto(InOut);
				while(1)
				{
					//ctrlproto_protocol_handler_function(pdo_out,pdo_in,InOut);			// read the incoming EtherCAT messages
					//torque_set = InOut.in_torque;
					input_torque <: 20;         	  									// torque control token
					input_torque <: torque_set;											// torque input

					#ifdef Quadrature_Encoder_used
						{speed_read, position_read, v} = get_qei_data(c_qei, q_max);	// QEI client to get the values from the QEI server

						// Output the sensor information back to EtherCAT //

						InOut.out_position = position_read;								// quadrature position output
						InOut.out_speed    = speed_read;								// quadrature speed output
					#else
						speed_read    =  get_hall_speed(sensor_output);					// Hall client to get the values from the Hall server
						position_read =  get_hall_angle(sensor_output);

						// Output the sensor information back to EtherCAT //

						InOut.out_position = position_read;								//hall position output
						InOut.out_speed    = speed_read;								//hall speed output
					#endif
				}
			}

		}


		/******************************************************************
		 * CORE 3          			Motor Control
		 ******************************************************************/
		on stdcore[IFM_CORE]: {
			par {
					adc_ad7949_triggered( c_adc, c_adctrig, clk_adc, p_ifm_adc_sclk_conv_mosib_mosia, p_ifm_adc_misoa, p_ifm_adc_misob);

					do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port, p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);

					/*********************************Hall Server*************************************/
					{
						init_hall(h_pole);										// initialize hall sensor
						run_hall_new(c_hall, sensor_output, p_ifm_hall, h_pole);	// start Hall Server
					}

					/****************************Motor Commutation loop*******************************/
					commutation_new(c_value, c_pwm_ctrl, sig);						// while(1) commutation loop

					/*******************************Torque Controller Loop****************************/
					{
						init_params_struct_all(t_params, f_params, loop_params);	// initialize controller parameter struct
		//				init_loop_pars(loop);										// initialize loop time
			//			init_torque_pars(t_params);									// initialize the torque controller loop
						foc_loop(sig, input_torque, c_adc, c_hall, c_value, t_params, f_params, loop_params);    // while(1) torque control loop
					}

					/**********************************QEI Server************************************/
					#ifdef Quadrature_Encoder_used
					{
						do_qei( c_qei, p_ifm_encoder );							// start Quadrature Encoder (QEI) Server
					}
					#endif
			  }
		}

	}

	return 0;

}


