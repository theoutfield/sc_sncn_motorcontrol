/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IMF_BOARD_REQUIRED" WIT A APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.inc>
#include <IFM_DC100-rev-b.inc>

/**
 * @file test_qei.xc
 * @brief Test illustrates usage of qei sensor to get position and velocity information
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <qei_client.h>
#include <qei_server.h>
#include <refclk.h>
#include <xscope.h>

/* Test QEI Sensor Client */
void qei_test(chanend c_qei)
{
	int position;
	int velocity;
	int direction;
	int core_id = 1;
	timer t;
	int count=0;
	qei_par qei_params;
	qei_velocity_par qei_velocity_params;  			// to compute velocity from qei
	init_qei_param(qei_params);
	init_qei_velocity_params(qei_velocity_params);

	while(1)
	{
		/* get position from QEI Sensor */
		{count, direction} = get_qei_position_absolute(c_qei);
		{position, direction} = get_qei_position(c_qei, qei_params);

		/* calculate velocity from QEI Sensor position */
		velocity = get_qei_velocity(c_qei, qei_params, qei_velocity_params);

		xscope_int(COUNT, count);
		xscope_int(POSITION, position);
		xscope_int(VELOCITY, velocity);

		wait_ms(1, core_id, t);
	}
}

int main(void)
{
	chan c_qei_p1;				// qei channel

	par
	{
		on tile[COM_TILE]:
		{
			/* Test QEI Sensor Client */
			qei_test(c_qei_p1);
		}

		/************************************************************
		 * IFM_TILE
		 ************************************************************/
		on tile[IFM_TILE]:
		{
			/* QEI Server Loop */
			{
				qei_par qei_params;
				run_qei(c_qei_p1, null, null, null, null, null, p_ifm_encoder, qei_params);  		// channel priority 1,2..6
			}
		}
	}

	return 0;
}
