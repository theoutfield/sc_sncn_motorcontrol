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


#include "profile_test.h"

#define ENABLE_xscope_main
#define COM_CORE 0
#define IFM_CORE 3


on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;
//internal
void set_velocity(int target_velocity, chanend c_velocity_ctrl)
{
	c_velocity_ctrl <: SET_VELOCITY_TOKEN;
	c_velocity_ctrl <: target_velocity;
}
int get_velocity(chanend c_velocity_ctrl)
{
	int velocity;
	c_velocity_ctrl <: GET_VELOCITY_TOKEN;
	c_velocity_ctrl :> velocity;
	return velocity;
}
int max_speed_limit(int velocity, int max_speed)
{
	if(velocity > max_speed)
	{
		velocity = max_speed;
		return velocity;
	}
	else if(velocity < -max_speed)
	{
		velocity = -max_speed;
		return velocity;
	}
	else if(velocity > -max_speed && velocity < max_speed)
	{
		return velocity;
	}
}
//csv mode function
void set_velocity_csv(csv_par &csv_params, int target_velocity, int velocity_offset, int torque_offset, chanend c_velocity_ctrl)
{
	set_velocity(max_speed_limit( (target_velocity + velocity_offset)*csv_params.polarity, csv_params.max_motor_speed), c_velocity_ctrl);
}




//basic velocity ctrl test
void vel_test(chanend c_velocity_ctrl)
{
	int velocity;
	in_data d;
	timer ts;
	unsigned int time;
	ts :> time;
		ts when timerafter(time+5*SEC_FAST) :> time;

	while (1)
	{
		input_vel(d);;
		printintln(d.set_velocity);
		set_velocity(d.set_velocity, c_velocity_ctrl);
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
	chan sig_1, signal_ctrl;
	chan c_velocity_ctrl;
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
		on stdcore[1]:
		{
			par
			{
				// test_stop(); //profile only

				//test_profile_velocity();
			//	vel_test(c_velocity_ctrl);

				//ethercat local test
				{
					int i;
					int u = 0, v_d = 2000, acc = 8000, dec =1050;
					int steps = 0, v_ramp;
					timer t; int time;
					csv_par csv_params;
					init_csv(csv_params);

					steps = init_velocity_profile(v_d, u*csv_params.polarity, acc, dec);
					printintln(steps);

					t:>time;	t when timerafter(time + 10*SEC_STD) :> time;
					printstrln("start");
					for(i = 1; i < steps; i++)
					{
						t when timerafter(time + 1*MSEC_STD) :> time;
						v_ramp = velocity_profile_generate(i);
					//	set_velocity(v_ramp, c_velocity_ctrl);
						set_velocity_csv(csv_params, v_ramp, 0, 0, c_velocity_ctrl);
						xscope_probe_data(2, v_ramp);
					//	xscope_probe_data(3, set_velocity_csv(csv_params, v_ramp, 0, 0, c_velocity_ctrl));
					}

				//	set_velocity(v_ramp, c_velocity_ctrl);
					set_velocity_csv(csv_params, v_ramp, 0, 0, c_velocity_ctrl);

					t when timerafter(time + 2*SEC_STD) :> time;
					u = get_velocity(c_velocity_ctrl); v_d = -2000;
					steps = init_velocity_profile(v_d, u*csv_params.polarity, acc, dec);

					for(i = 1; i < steps; i++)
					{
						t when timerafter(time + 1*MSEC_STD) :> time;
						v_ramp = velocity_profile_generate(i);
					//	set_velocity(v_ramp, c_velocity_ctrl);
						set_velocity_csv(csv_params, v_ramp, 0, 0, c_velocity_ctrl);
						xscope_probe_data(2, v_ramp);
					//	xscope_probe_data(3, set_velocity_csv(csv_params, v_ramp, 0, 0, c_velocity_ctrl));
					}
					//set_velocity(v_ramp, c_velocity_ctrl);
					set_velocity_csv(csv_params, v_ramp, 0, 0, c_velocity_ctrl);
										t when timerafter(time + 2*SEC_STD) :> time;

					u = get_velocity(c_velocity_ctrl); v_d = 2000;
					steps = init_velocity_profile(v_d, u*csv_params.polarity, acc, dec);

					for(i = 1; i < steps; i++)
					{
						t when timerafter(time + 1*MSEC_STD) :> time;
						v_ramp = velocity_profile_generate(i);
					//	set_velocity(v_ramp, c_velocity_ctrl);
						set_velocity_csv(csv_params, v_ramp, 0, 0, c_velocity_ctrl);
						xscope_probe_data(2, v_ramp);
					//	xscope_probe_data(3, set_velocity_csv(csv_params, v_ramp, 0, 0, c_velocity_ctrl));
					}

					t when timerafter(time + 4*SEC_STD) :> time;

					u = get_velocity(c_velocity_ctrl);
					steps = init_quick_stop_velocity_profile(u*csv_params.polarity, dec);

					for(i = 1; i < steps; i++)
					{
						t when timerafter(time + 1*MSEC_STD) :> time;
						v_ramp = quick_stop_velocity_profile_generate(i);
					//	set_velocity(v_ramp, c_velocity_ctrl);
						set_velocity_csv(csv_params, v_ramp, 0, 0, c_velocity_ctrl);
						xscope_probe_data(2, v_ramp);
					//	xscope_probe_data(3, set_velocity_csv(csv_params, v_ramp, 0, 0, c_velocity_ctrl));
					}

				//	printstrln("end");
					while(1)
					{
						//set_velocity(v_ramp, c_velocity_ctrl);
						set_velocity_csv(csv_params, v_ramp, 0, 0, c_velocity_ctrl);
					}
				}

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
					ctrl_par velocity_ctrl_params;
					filt_par sensor_filter_params;
					hall_par hall_params;
					qei_par qei_params;


					init_velocity_control(velocity_ctrl_params);
					init_sensor_filter(sensor_filter_params);
					init_hall(hall_params);
					init_qei(qei_params);

				//	while(1)
				//	set_commutation_sinusoidal(c_commutation, 1500);

					velocity_control(velocity_ctrl_params, sensor_filter_params, hall_params, qei_params, 2, c_hall_p2, c_qei, c_velocity_ctrl, c_commutation);
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

				commutation_sinusoidal(c_commutation, c_hall_p1, c_pwm_ctrl, signal_adc); 	// hall based sinusoidal commutation


				run_hall( p_ifm_hall, c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4);  		// channel priority 1,2..4

				run_qei(c_qei, p_ifm_encoder);

			}
		}

	}

	return 0;
}
