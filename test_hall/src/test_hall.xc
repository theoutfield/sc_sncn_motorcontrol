
/**
 * \file test_hall.xc
 * \brief Test illustrates usage of hall sensor to get position and velocity information
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \author Martin Schwarz <mschwarz@synapticon.com>
 */

#include <xs1.h>
#include <platform.h>
#include <print.h>
#include <ioports.h>
#include <hall_client.h>
#include <hall_server.h>
#include <refclk.h>
#include <bldc_motor_config.h>

#define TILE_ONE 0
#define IFM_TILE 3


/* Test Hall Sensor Client */
void hall_test(chanend c_hall)
{
	int position;
	int velocity;
	int core_id = 1;
	timer t;
	int direction;
	hall_par hall_params;
	init_hall_param(hall_params);

	while(1)
	{
		/* get position from Hall Sensor */
		{position, direction} = get_hall_position_absolute(c_hall);

		/* get velocity from Hall Sensor */
		velocity = get_hall_velocity(c_hall, hall_params);

		printstr("Position: ");
		printint(position);
		printstr(" ");
		printstr("Velocity: ");
		printintln(velocity);
	}
}

int main(void)
{
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6;	// hall channels

	par
	{
		on stdcore[TILE_ONE]:
		{
			/* Test Hall Sensor Client */
			hall_test(c_hall_p1);

		}

		/************************************************************
		 * IFM_TILE
		 ************************************************************/
		on stdcore[IFM_TILE]:
		{
			/* Hall Server */
			{
				hall_par hall_params;
				init_hall_param(hall_params);
				run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6, p_ifm_hall, hall_params); // channel priority 1,2..6
			}

		}

	}

	return 0;
}
