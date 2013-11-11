/*
 *
 * \file
 * \brief Main project file
 *
 * Port declarations, etc.
 *
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 0.1 (2012-11-23 1850)
 *
 */

#include <xs1.h>
#include <platform.h>
#include <print.h>
#include <stdio.h>
#include <stdint.h>
#include <ioports.h>
#include <xscope.h>
#include <adc_server_ad7949.h>
#include <qei_server.h>
#include <hall_server.h>
#include <flash_somanet.h>
#include <pwm_service_inv.h>
#include <comm_loop.h>
#include <velocity_ctrl_server.h>
#include <position_ctrl_server.h>
#include <torque_ctrl_server.h>
#include <ecat_motor_drive.h>
#include <dc_motor_config.h>

#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;

int main(void) {
	chan c_adc, c_adctrig;
	chan c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5 ;
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5;
	chan c_commutation_p1, c_commutation_p2, c_commutation_p3;
	chan c_pwm_ctrl;
	chan c_signal_adc;
	chan c_sig_1, c_signal, c_sync;
	chan c_velocity_ctrl;
	chan c_torque_ctrl;
	chan c_position_ctrl;

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


	par
	{
		on stdcore[0] :
		{
			ecat_init();
			ecat_handler(coe_out, coe_in, eoe_out, eoe_in, eoe_sig, foe_out,
					foe_in, pdo_out, pdo_in);
		}

		on stdcore[0] :
		{
			//firmware_update(foe_out, foe_in, c_sig_1); // firmware update
		}

		on stdcore[1] :
		{
			ecat_motor_drive(pdo_out, pdo_in, coe_out, c_signal, c_hall_p5, c_qei_p5, c_torque_ctrl, c_velocity_ctrl, c_position_ctrl);
		}

		on stdcore[2]:
		{
			par
			{
				{
					 ctrl_par position_ctrl_params;
					 hall_par hall_params;
					 qei_par qei_params;

					 init_position_control_param(position_ctrl_params);
					 init_hall_param(hall_params);
					 init_qei_param(qei_params);

					 position_control(position_ctrl_params, hall_params, qei_params, 2, c_hall_p4,\
							 c_qei_p4, c_position_ctrl, c_commutation_p3);
				}

				{
					 ctrl_par velocity_ctrl_params;
					 filter_par sensor_filter_params;
					 hall_par hall_params;
					 qei_par qei_params;

					 init_velocity_control_param(velocity_ctrl_params);
					 init_sensor_filter_param(sensor_filter_params);
					 init_hall_param(hall_params);
					 init_qei_param(qei_params);

					 velocity_control(velocity_ctrl_params, sensor_filter_params, hall_params,\
							 qei_params, 2, c_hall_p3, c_qei_p3, c_velocity_ctrl, c_commutation_p2);
				 }


				{
					ctrl_par torque_ctrl_params;
					hall_par hall_params;
					qei_par qei_params;

					init_qei_param(qei_params);
					init_hall_param(hall_params);
					init_torque_control_param(torque_ctrl_params);

					torque_control( torque_ctrl_params, hall_params, qei_params, c_adc, \
							c_commutation_p1,  c_hall_p2,  c_qei_p2, c_torque_ctrl);
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

				{
					hall_par hall_params;
					qei_par qei_params;
					commutation_par commutation_params;
					init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED); // initialize commutation params
					init_hall_param(hall_params);
				//	comm_init_ecat(c_signal, hall_params);

					init_qei_param(qei_params);

					commutation_sinusoidal(c_hall_p1,  c_qei_p1,
								 c_signal, c_sync, c_commutation_p1, c_commutation_p2,
								 c_commutation_p3, c_pwm_ctrl, hall_params,
								 qei_params, commutation_params);

				}

				{
					hall_par hall_params;
					init_hall_param(hall_params);
			//		hall_init_ecat(c_hall_p5, hall_params);   	//same as ecat drive channel

					run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, p_ifm_hall, hall_params); // channel priority 1,2..4
				}

				{
					qei_par qei_params;
					init_qei_param(qei_params);
			//		qei_init_ecat(c_qei_p5, qei_params);  		//same as ecat drive channel

					run_qei(c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5, p_ifm_encoder, qei_params);  // channel priority 1,2..4
				}

			}
		}

	}

	return 0;
}
