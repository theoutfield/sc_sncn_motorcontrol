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

#include <torque_ctrl_service.h>
#include <profile_control.h>
#include <profile.h>

#include <xscope.h>
//Configure your motor parameters in config/bldc_motor_config.h
#include <bldc_motor_config.h>
#include <qei_config.h>

/* Test Profile Torque Function */
void profile_torque_test(interface TorqueControlInterface client i_torque_control)
{
    delay_seconds(1);

	int target_torque = 500; 	//(desired torque/torque_constant)  * IFM resolution
	int torque_slope  = 500;  	//(desired torque_slope/torque_constant)  * IFM resolution
	cst_par cst_params; int actual_torque; timer t; unsigned int time;
	init_cst_param(cst_params);
    xscope_int(TARGET_TORQUE, target_torque);

	/* Set new target torque for profile torque control */
	set_profile_torque( target_torque, torque_slope, cst_params, i_torque_control);

	delay_seconds(5);

	target_torque = 0;
	xscope_int(TARGET_TORQUE, target_torque);
	set_profile_torque( target_torque, torque_slope, cst_params, i_torque_control);

	target_torque = -500;
	xscope_int(TARGET_TORQUE, target_torque);
	set_profile_torque( target_torque, torque_slope, cst_params, i_torque_control);
	t:>time;

	while(1)
	{
		actual_torque = i_torque_control.get_torque()*cst_params.polarity;

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
QEIPorts encoder_ports = ENCODER_PORTS;

int main(void)
{
	// Motor control channels
	chan c_adctrig, c_pwm_ctrl;

	interface WatchdogInterface i_watchdog;
    interface CommutationInterface i_commutation[3];
    interface ADCInterface adc_interface;
    interface HallInterface i_hall[5];
    interface QEIInterface i_qei[5];

    interface TorqueControlInterface i_torque_control;

	par
	{
		/* Test Profile Torque Function */
		on tile[0]: profile_torque_test(i_torque_control);

		on tile[2]:
		{
			par
			{
				/* Torque Control Loop */
				{
					ctrl_par torque_ctrl_params;
                    init_torque_control_param(torque_ctrl_params);  /* Initialize PID parameters for Torque Control (defined in config/motor/bldc_motor_config.h) */

					HallConfig hall_config;
					init_hall_config(hall_config);

					QEIConfig qei_params;
		            init_qei_config(qei_params); /* Initialize Sensor configuration parameters (defined in config/motor/bldc_motor_config.h) */

					/* Control Loop */
					torque_control_service( torque_ctrl_params, hall_config, qei_params, SENSOR_USED,
					        adc_interface, i_commutation[0],  i_hall[1], i_qei[1], i_torque_control);
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
                {
                	HallConfig hall_config;
                	//NEEDS INITIALIZATION
                	hall_service(i_hall, hall_ports, hall_config);
            	}


				/* Motor Commutation loop */
				{
					HallConfig hall_config;
					QEIConfig qei_params;
					commutation_par commutation_params;
					init_hall_config(hall_config);
					init_qei_config(qei_params);

					commutation_service(i_hall[0],  i_qei[0], null, i_watchdog, i_commutation, c_pwm_ctrl,
					        fet_driver_ports, hall_config, qei_params, commutation_params);
				}

				/* QEI Server */
				{
                    QEIConfig qei_config;
                    init_qei_config(qei_config);

                    qei_service(i_qei, encoder_ports, qei_config);         // channel priority 1,2..6
				}
			}
		}

	}

	return 0;
}
