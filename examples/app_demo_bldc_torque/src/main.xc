/* INCLUDE BOARD SUPPORT FILES FROM module_board-support */
#include <CORE_C22-rev-a.inc>
#include <IFM_DC100-rev-b.inc>

/**
 * @file test_torque-ctrl.xc
 * @brief Test illustrates usage of profile torque control
 * @author Synapticon GmbH (www.synapticon.com)
 */

#include <print.h>
#include <refclk.h>

#include <hall_service.h>
#include <qei_service.h>
#include <pwm_service_inv.h>
#include <adc_service.h>
#include <commutation_service.h>

#include <torque_ctrl_server.h>
#include <profile_control.h>
#include <torque_ctrl_client.h>
#include <profile.h>

#include <xscope.h>
//Configure your motor parameters in config/bldc_motor_config.h
#include <bldc_motor_config.h>

#ifdef DC1K
port p_ifm_encoder_hall_select_ext_d4to5 = SELECTION_HALL_ENCODER_PORT;
#endif

/* Test Profile Torque Function */
void profile_torque_test(chanend c_torque_ctrl)
{
    delay_seconds(1);

	int target_torque = 500; 	//(desired torque/torque_constant)  * IFM resolution
	int torque_slope  = 500;  	//(desired torque_slope/torque_constant)  * IFM resolution
	cst_par cst_params; int actual_torque; timer t; unsigned int time;
	init_cst_param(cst_params);
    xscope_int(TARGET_TORQUE, target_torque);

	/* Set new target torque for profile torque control */
	set_profile_torque( target_torque, torque_slope, cst_params, c_torque_ctrl);

	delay_seconds(5);

	target_torque = 0;
	xscope_int(TARGET_TORQUE, target_torque);
	set_profile_torque( target_torque, torque_slope, cst_params, c_torque_ctrl);

	target_torque = -1000;
	xscope_int(TARGET_TORQUE, target_torque);
	set_profile_torque( target_torque, torque_slope, cst_params, c_torque_ctrl);
	t:>time;

	while(1)
	{
		actual_torque = get_torque(c_torque_ctrl)*cst_params.polarity;

        xscope_int(ACTUAL_TORQUE, actual_torque);
        xscope_int(TARGET_TORQUE, target_torque);

		t when timerafter(time + MSEC_STD) :> time;
	}
}

PwmPorts pwm_ports = PWM_PORTS;
WatchdogPorts wd_ports = WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = FET_DRIVER_PORTS;
ADCPorts adc_ports = ADC_PORTS;
HallPorts hall_ports = HALL_PORTS;
EncoderPorts encoder_ports = ENCODER_PORTS;

int main(void)
{
	// Motor control channels
	chan c_adctrig;													// adc channels
	chan c_signal;	                                                        // commutation channels
	chan c_pwm_ctrl;														// pwm channel
	chan c_torque_ctrl;                                 					// torque control channel

	interface WatchdogInterface i_watchdog;
    interface CommutationInterface i_commutation[3];
    interface ADCInterface adc_interface;
    interface HallInterface i_hall[5];
    interface QEIInterface i_qei[5];

	par
	{
		/* Test Profile Torque Function */
		on tile[0]: profile_torque_test(c_torque_ctrl);

		on tile[2]:
		{
			par
			{
				/* Torque Control Loop */
				{
					ctrl_par torque_ctrl_params;
					hall_par hall_params;
					qei_par qei_params;

					/* Initialize PID parameters for Torque Control (defined in config/motor/bldc_motor_config.h) */
					init_torque_control_param(torque_ctrl_params);

					/* Initialize Sensor configuration parameters (defined in config/motor/bldc_motor_config.h) */
					init_qei_param(qei_params);
					init_hall_param(hall_params);

					/* Control Loop */
					torque_control( torque_ctrl_params, hall_params, qei_params, SENSOR_USED,
					        adc_interface, i_commutation[0],  i_hall[1], i_qei[1], c_torque_ctrl);
				}
			}
		}

		/************************************************************
		 * IFM_TILE
		 ************************************************************/
		on tile[IFM_TILE]:
		{
			par
			{
				/* ADC Loop */
			    adc_service(adc_interface, adc_ports, c_adctrig);

				/* PWM Loop */
				do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, pwm_ports);

                /* Watchdog Server */
                watchdog_service(i_watchdog, wd_ports);

                /* Hall Server */
                hall_service(i_hall, hall_ports); // channel priority 1,2..4

				/* Motor Commutation loop */
				{
					hall_par hall_params;
					qei_par qei_params;
					commutation_par commutation_params;
					init_hall_param(hall_params);
					init_qei_param(qei_params);
					commutation_service(i_hall[0],  i_qei[0], c_signal, i_watchdog, i_commutation, c_pwm_ctrl,
					        fet_driver_ports, hall_params, qei_params, commutation_params);
				}

				/* QEI Server */
				{
                    qei_velocity_par qei_velocity_params;
                    qei_par qei_config;
                    init_qei_velocity_params(qei_velocity_params);

                    qei_service(i_qei, encoder_ports, qei_config, qei_velocity_params);         // channel priority 1,2..6
				}
			}
		}

	}

	return 0;
}
