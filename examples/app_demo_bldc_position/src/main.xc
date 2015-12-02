/* INCLUDE BOARD SUPPORT FILES FROM module_board-support */
#include <CORE_C22-rev-a.inc>
#include <IFM_DC100-rev-b.inc>

/**
 * @file test_position-ctrl.xc
 * @brief Test illustrates usage of profile position control
 * @author Synapticon GmbH (www.synapticon.com)
 */

//BLDC Motor drive libs
#include <qei_service.h>
#include <hall_service.h>
#include <pwm_service.h>
#include <commutation_service.h>

//Position control + profile libs
#include <position_ctrl_service.h>
#include <profile_control.h>

//Configuration headers
#include <bldc_motor_config.h>
#include <qei_config.h>
#include <hall_config.h>
#include <commutation_config.h>
#include <control_config.h>

/* Test Profile Position function */
void position_profile_test(interface PositionControlInterface client i_position_control)
{
	int actual_position = 0;			// ticks
	int target_position = 16000;		// HALL: 4096 extrapolated ticks x nr. pole pairs = one rotation; QEI: your encoder documented resolution x 4 = one rotation
	int velocity 		= 500;			// rpm
	int acceleration 	= 500;			// rpm/s
	int deceleration 	= 500;     	// rpm/s
	int follow_error;

	/* Initialise the position profile generator */
	init_position_profiler(MIN_POSITION_LIMIT, MAX_POSITION_LIMIT, MAX_PROFILE_VELOCITY, MAX_ACCELERATION, i_position_control);

	/* Set new target position for profile position control */
	set_profile_position(target_position, velocity, acceleration, deceleration, i_position_control);

	while(1)
	{
	    /* Read actual position from the Position Control Server */
		actual_position = i_position_control.get_position();
		follow_error = target_position - actual_position;

		xscope_int(ACTUAL_POSITION, actual_position);
		xscope_int(TARGET_POSITION, target_position);
		xscope_int(FOLLOW_ERROR, follow_error);

		delay_milliseconds(1); /* 1 ms wait */
	}
}

PwmPorts pwm_ports = PWM_PORTS;
WatchdogPorts wd_ports = WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = FET_DRIVER_PORTS;
HallPorts hall_ports = HALL_PORTS;
QEIPorts encoder_ports = ENCODER_PORTS;

int main(void)
{
	// Motor control channels
	chan c_pwm_ctrl;			// pwm channel

	interface WatchdogInterface i_watchdog;
	interface CommutationInterface i_commutation[5];
	interface HallInterface i_hall[5];
	interface QEIInterface i_qei[5];

	interface PositionControlInterface i_position_control;

	par
	{
		/* Test Profile Position Client function*/
		on tile[APP_TILE_1]: position_profile_test(i_position_control);      // test PPM on slave side

		/************************************************************
		 * IFM_TILE
		 ************************************************************/
		on tile[IFM_TILE]:
		{
			par
			{
				/* PWM Loop */
                pwm_service(c_pwm_ctrl, pwm_ports);

                /* Watchdog Server */
                watchdog_service(i_watchdog, wd_ports);

                /* QEI Service */
                {
                    QEIConfig qei_config;
                    init_qei_config(qei_config);

                    qei_service(i_qei, encoder_ports, qei_config);
                }

                {
                	HallConfig hall_config;
                	init_hall_config(hall_config);

                	hall_service(i_hall, hall_ports, hall_config);
            	}

				/* Motor Commutation loop */
				{
				    CommutationConfig commutation_config;
				    init_commutation_config(commutation_config);

					commutation_service(i_hall[0], i_qei[0], i_watchdog, i_commutation,
					                        c_pwm_ctrl, fet_driver_ports, commutation_config);
				}

                /* Position Control Loop */
                {
                     ControlConfig position_ctrl_params;
                     init_position_control_config(position_ctrl_params); // Initialize PID parameters for Position Control

                     /* Control Loop */
                     position_control_service(position_ctrl_params, i_hall[1], i_qei[1],
                                               i_position_control, i_commutation[0]);
                }
            }
        }
    }

	return 0;
}
