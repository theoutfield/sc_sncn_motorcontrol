/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_BOARD_REQUIRED>
#include <IFM_BOARD_REQUIRED>


#include <xs1.h>
#include <platform.h>
#include <stdio.h>
#include <hall_server.h>
#include <qei_server.h>
#include <biss_server.h>
#include <pwm_service_inv.h>
#include <brushed_dc_server.h>
#include <brushed_dc_client.h>
#include <refclk.h>
#include <velocity_ctrl_client.h>
#include <velocity_ctrl_server.h>
#include <internal_config.h>
#include <drive_modes.h>
#include <statemachine.h>
#include <drive_modes.h>
#include <statemachine.h>
#include <profile_control.h>
#include <position_ctrl_server.h>
#include <drive_modes.h>
#include <position_ctrl_client.h>
#include <qei_client.h>
#include <profile.h>
#include <bldc_motor_config.h>
#include <watchdog.h>

//#define ENABLE_xscope

on tile[IFM_TILE]: clock clk_adc = XS1_CLKBLK_1;
on tile[IFM_TILE]: clock clk_pwm = XS1_CLKBLK_REF;
on tile[IFM_TILE]: clock clk_biss = XS1_CLKBLK_2 ;
port out p_ifm_biss_clk = GPIO_D0;

/* Test Profile Position function */
void position_profile_test(chanend c_position_ctrl, chanend c_qei, chanend c_hall)
{
	int actual_position = 0;			// ticks
	int target_position = GEAR_RATIO*ENCODER_RESOLUTION*5;			// ticks
	int velocity 		= 500;			// rpm
	int acceleration 	= 100;			// rpm/s
	int deceleration 	= 100;     		// rpm/s
	int following_error;
	timer t;
	hall_par hall_params;
	qei_par qei_params;
	init_qei_param(qei_params);
	init_hall_param(hall_params);

	/* Initialise Profile Limits for position profile generator and select position sensor */
	init_position_profile_limits(MAX_ACCELERATION, MAX_PROFILE_VELOCITY, qei_params, hall_params, \
			SENSOR_USED, MAX_POSITION_LIMIT, MIN_POSITION_LIMIT);

	int init = init_position_control(c_position_ctrl);

	/* Set new target position for profile position control */
	set_profile_position(target_position, velocity, acceleration, deceleration, SENSOR_USED, c_position_ctrl);

	/* Read actual position from the Position Control Server */
	actual_position = get_position(c_position_ctrl);

	while(1)
	{
		actual_position = get_position(c_position_ctrl);
		following_error = target_position - actual_position;
		printf("actual position: %i  |  following error: %i\n", actual_position, following_error);
	}
}

int main(void)
{
	// Motor control channels
	chan c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5, c_hall_p6, c_qei_p6;		// qei channels
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5;						// hall channels
	chan c_commutation;						// motor drive channels
	chan c_pwm_ctrl, c_adctrig;														// pwm channels
	chan c_position_ctrl;															// position control channel
	chan c_watchdog; 																// watchdog channel
	interface i_biss i_biss[1];                                                     // biss interface

	par
	{

		/* Test Profile Position Client function*/
		on tile[0]:
		{
			position_profile_test(c_position_ctrl, c_qei_p5, c_hall_p5);		// test PPM on slave side
		}


		on tile[0]:
		{
			/* Position Control Loop */
			{
				 ctrl_par position_ctrl_params;
				 hall_par hall_params;
				 qei_par qei_params;

				 /* Initialize PID parameters for Position Control (defined in config/motor/bldc_motor_config.h) */
				 init_position_control_param(position_ctrl_params);

				 /* Initialize Sensor configuration parameters (defined in config/motor/bldc_motor_config.h) */
				 init_hall_param(hall_params);
				 init_qei_param(qei_params);

				 /* Control Loop */
				 position_control(position_ctrl_params, hall_params, qei_params, SENSOR_USED, c_hall_p2,\
						 c_qei_p2, i_biss[0], c_position_ctrl, c_commutation);
			}
		}

		/************************************************************
		 * IFM_TILE
		 ************************************************************/
		on tile[IFM_TILE]:
		{
			par
			{
				/* PWM Loop */
				do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port,\
						p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);

				/* Brushed Motor Drive loop */
				{
					bdc_loop(c_watchdog, c_commutation, c_pwm_ctrl,\
							p_ifm_esf_rstn_pwml_pwmh, p_ifm_coastn, p_ifm_ff1, p_ifm_ff2);
				}

				/* Watchdog Server */
				run_watchdog(c_watchdog, p_ifm_wd_tick, p_ifm_shared_leds_wden);

				/* Hall Server */
				{
					hall_par hall_params;
					run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6, p_ifm_hall, hall_params); // channel priority 1,2..5
				}

#if (SENSOR_USED != BISS)
				/* QEI Server */
				{
					qei_par qei_params;
					run_qei(c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5, c_qei_p6, p_ifm_encoder, qei_params);  		 // channel priority 1,2..5
				}
#else
				/* biss server */
				{
				    biss_par biss_params;
				    run_biss(i_biss, 1, p_ifm_biss_clk, p_ifm_encoder, clk_biss, biss_params, 2);
				}
#endif

			}
		}

	}

	return 0;
}
