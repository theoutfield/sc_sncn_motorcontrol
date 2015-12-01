/* INCLUDE BOARD SUPPORT FILES FROM module_board-support */
#include <CORE_C22-rev-a.inc>
#include <IFM_DC100-rev-b.inc>

/**
 * @file test_torque-ctrl.xc
 * @brief Test illustrates usage of profile torque control
 * @author Synapticon GmbH (www.synapticon.com)
 */

//BLDC Motor drive libs
#include <qei_service.h>
#include <hall_service.h>
#include <pwm_service.h>
#include <adc_service.h>
#include <commutation_service.h>

//Torque control + profile libs
#include <torque_ctrl_service.h>
#include <profile_control.h>

//Configure your motor parameters in config/bldc_motor_config.h
#include <bldc_motor_config.h>
#include <qei_config.h>
#include <hall_config.h>
#include <commutation_config.h>
#include <control_config.h>

/* Test Profile Torque Function */
void profile_torque_test(interface TorqueControlInterface client i_torque_control)
{
	int target_torque = 500; 	//(desired torque/torque_constant)  * IFM resolution
	int torque_slope  = 500;  	//(desired torque_slope/torque_constant)  * IFM resolution
	int actual_torque;
    xscope_int(TARGET_TORQUE, target_torque);


	/* Initialise the torque profile generator */
	init_torque_profiler(MOTOR_TORQUE_CONSTANT * MAX_NOMINAL_CURRENT * IFM_RESOLUTION, POLARITY, i_torque_control);

	/* Set new target torque for profile torque control */
	set_profile_torque( target_torque, torque_slope, i_torque_control);

	delay_seconds(3);

	target_torque = -500;
	xscope_int(TARGET_TORQUE, target_torque);

    /* Set new target torque for profile torque control */
	set_profile_torque( target_torque, torque_slope, i_torque_control);

	while(1)
	{
		actual_torque = i_torque_control.get_torque();

        xscope_int(ACTUAL_TORQUE, actual_torque);
        xscope_int(TARGET_TORQUE, target_torque);

		delay_milliseconds(1);
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
    interface ADCInterface i_adc;
    interface HallInterface i_hall[5];
    interface QEIInterface i_qei[5];

    interface TorqueControlInterface i_torque_control;

	par
	{
		/* Test Profile Torque Function */
		on tile[APP_TILE]: profile_torque_test(i_torque_control);

		on tile[APP_TILE]:
		{
		    /* Torque Control Loop */
            ControlConfig torque_ctrl_params;
            init_torque_control_config(torque_ctrl_params);  // Initialize PID parameters for Torque Control

            /* Control Loop */
            torque_control_service( torque_ctrl_params, i_adc, i_commutation[0],  i_hall[1], i_qei[1], i_torque_control);
        }


		/************************************************************
		 * IFM_TILE
		 ************************************************************/
		on tile[IFM_TILE]:
		{
			par
			{
				/* PWM Loop */
				pwm_triggered_service(c_pwm_ctrl, c_adctrig, pwm_ports);

                /* Watchdog Server */
                watchdog_service(i_watchdog, wd_ports);

                /* ADC Loop */
                adc_service(i_adc, adc_ports, c_adctrig);

                {
                    HallConfig hall_config;
                    init_hall_config(hall_config);

                    hall_service(i_hall, hall_ports, hall_config);
                }

                /* QEI Server */
                {
                    QEIConfig qei_config;
                    init_qei_config(qei_config);

                    qei_service(i_qei, encoder_ports, qei_config);
                }

				/* Motor Commutation loop */
                {
                    CommutationConfig commutation_config;
                    init_commutation_config(commutation_config);

                    commutation_service(i_hall[0], i_qei[0], null, i_watchdog, i_commutation,
                            c_pwm_ctrl, fet_driver_ports, commutation_config);
                }
			}
		}

	}

	return 0;
}
