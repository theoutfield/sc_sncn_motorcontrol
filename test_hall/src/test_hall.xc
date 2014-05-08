
/**
 * \file test_hall.xc
 * \brief Test illustrates usage of hall sensor to get position and velocity information
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */

#include <xs1.h>
#include <platform.h>
#include <print.h>
#include <ioports.h>
#include <hall_client.h>
#include <hall_server.h>
#include <refclk.h>
#include <xscope.h>
#include <bldc_motor_config.h>

//#define ENABLE_xscope
#define COM_CORE 0
#define IFM_CORE 3


void xscope_initialise_1(void)
{
    xscope_register(2,
            XSCOPE_CONTINUOUS, "0 hall_position", XSCOPE_INT,	"n",
            XSCOPE_CONTINUOUS, "1 hall_velocity", XSCOPE_INT,	"n");
}


/* Test Hall Sensor Client */
void hall_test(chanend c_hall)
{
	int position;
	int velocity;
//	int core_id = 1;
//	timer t;
	int direction;
	hall_par hall_params;
	init_hall_param(hall_params);

#ifdef ENABLE_xscope
	xscope_initialise_1();
#endif

	while(1)
	{
		/* get position from Hall Sensor */
		{position, direction} = get_hall_position_absolute(c_hall);

		/* get velocity from Hall Sensor */
		velocity = get_hall_velocity(c_hall, hall_params);

#ifdef ENABLE_xscope
		xscope_core_int(0, position);
		xscope_core_int(1, velocity);
#else
		printstr("Position: ");
		printint(position);
		printstr(" ");
		printstr("Velocity: ");
		printintln(velocity);
#endif

	}
}

int main(void)
{
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6;	// hall channels

	par
	{
		on tile[0]:
		{
			/* Test Hall Sensor Client */
			par
			{
				hall_test(c_hall_p1);
			   /* {
			        int i = 0;
			        timer t;
			        xscope_initialise_1();
			        while(1)
			        {
			          //  printintln(i);
			            //i = (i + 1) % 100;

			            //wait_ms(1, 1, t);
			           // xscope_int(0, i);
			        }

			    }*/
			}
		}

		/************************************************************
		 * IFM_CORE
		 ************************************************************/
		on tile[IFM_CORE]:
		{
			par
			{
				/* Hall Server */
				{
					hall_par hall_params;
					init_hall_param(hall_params);
					run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6, p_ifm_hall, hall_params); // channel priority 1,2..6
				}
			}
		}

	}

	return 0;
}
