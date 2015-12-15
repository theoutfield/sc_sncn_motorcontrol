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
#include <watchdog_service.h>
#include <motorcontrol_service.h>

//Position control + profile libs
#include <position_ctrl_service.h>
#include <profile_control.h>

//Configuration headers
#include <motorcontrol_config.h>
#include <control_config.h>

/* Test Profile Position function */
void position_profile_test(interface PositionControlInterface client i_position_control)
{
	int target_position = 16000;		// HALL: 1 rotation = 4096 x nr. pole pairs; QEI: your encoder documented resolution x 4 = one rotation
	int velocity 		= 500;			// rpm
	int acceleration 	= 500;			// rpm/s
	int deceleration 	= 500;     	    // rpm/s

	ProfilePositionConfig profile_position_config;
	profile_position_config.max_acceleration = MAX_ACCELERATION;
	profile_position_config.software_position_limit_max = MAX_POSITION_LIMIT;
	profile_position_config.software_position_limit_min = MIN_POSITION_LIMIT;
	profile_position_config.velocity_config.max_profile_velocity = MAX_PROFILE_VELOCITY;

	/* Initialise the position profile generator */
	init_position_profiler(profile_position_config, i_position_control);

	/* Set new target position for profile position control */
	set_profile_position(target_position, velocity, acceleration, deceleration, i_position_control);
}

PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;
HallPorts hall_ports = SOMANET_IFM_HALL_PORTS;
QEIPorts qei_ports = SOMANET_IFM_QEI_PORTS;

int main(void)
{
	// Motor control channels
	chan c_pwm_ctrl;			// pwm channel

	interface WatchdogInterface i_watchdog[2];
	interface MotorcontrolInterface i_motorcontrol[5];
	interface HallInterface i_hall[5];
	interface QEIInterface i_qei[5];

	interface PositionControlInterface i_position_control[3];

	par
	{
		/* Test Profile Position Client function*/
		on tile[APP_TILE]: position_profile_test(i_position_control[0]);      // test PPM on slave side

		/* XScope monitoring */
		on tile[APP_TILE]: {

		    int actual_position, target_position, follow_error;

		    while(1)
		    {
		        /* Read actual position from the Position Control Server */
		        actual_position = i_position_control[1].get_position();
		        target_position = i_position_control[1].get_target_position();

		        xscope_int(TARGET_POSITION, target_position/10); //Divided by 10 for better displaying
		        xscope_int(ACTUAL_POSITION, actual_position/10); //Divided by 10 for better displaying

		        delay_milliseconds(1); /* 1 ms wait */
		    }
		}

		on tile[APP_TILE]:
        /* Position Control Loop */
        {
             ControlConfig position_ctrl_params;
             init_position_control_config(position_ctrl_params); // Initialize PID parameters for Position Control

             /* Control Loop */
             position_control_service(position_ctrl_params, i_hall[1], i_qei[1], i_motorcontrol[0],
                                         i_position_control);
        }

		/************************************************************
		 * IFM_TILE
		 ************************************************************/
		on tile[IFM_TILE]:
		{
			par
			{
				/* PWM Loop */
                pwm_service(pwm_ports, c_pwm_ctrl);

                /* Watchdog Server */
                watchdog_service(wd_ports, i_watchdog);

                /* Hall sensor Service */
                {
                    HallConfig hall_config;
                        hall_config.pole_pairs = POLE_PAIRS;

                    hall_service(hall_ports, hall_config, i_hall);
                }

                /* Quadrature encoder sensor Service */
                {
                    QEIConfig qei_config;
                        qei_config.signal_type = QEI_SIGNAL_TYPE;               // Encoder signal type (just if applicable)
                        qei_config.index_type = QEI_INDEX_TYPE;                 // Indexed encoder?
                        qei_config.ticks_resolution = ENCODER_RESOLUTION;       // Encoder resolution
                        qei_config.sensor_polarity = QEI_SENSOR_POLARITY;       // CW

                    qei_service(qei_ports, qei_config, i_qei);
                }

				/* Motor Commutation loop */
				{
				    MotorcontrolConfig motorcontrol_config;
				    init_motorcontrol_config(motorcontrol_config);

					motorcontrol_service(fet_driver_ports, motorcontrol_config,
					                        c_pwm_ctrl, i_hall[0], i_qei[0], i_watchdog[0], i_motorcontrol);
				}

            }
        }
    }

	return 0;
}
