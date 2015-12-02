/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IMF_BOARD_REQUIRED" WIT A APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.inc>
#include <IFM_DC100-rev-b.inc>

/**
 * @file test_qei.xc
 * @brief Test illustrates usage of qei sensor to get position and velocity information
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <qei_service.h>

/* Test QEI Sensor Client */
void qei_test(interface QEIInterface client i_qei)
{
	int position;
	int velocity;
	int direction;
	int core_id = 1;
	int count=0;
	timer t;

	while(1)
	{
		/* get position and velocity from QEI Sensor */
		{count, direction} = i_qei.get_qei_position_absolute();
		{position, direction} = i_qei.get_qei_position();

		velocity = i_qei.get_qei_velocity();

		xscope_int(COUNT, count);
		xscope_int(POSITION, position);
		xscope_int(VELOCITY, velocity);

		delay_milliseconds(1);//, core_id, t);
	}
}

QEIPorts qei_ports = SOMANET_IFM_QEI_PORTS;

int main(void)
{
    interface QEIInterface i_qei[5];

	par
	{
		on tile[APP_TILE]:
		{
			/* Test QEI Sensor Client */
			qei_test(i_qei[0]);
		}

		/************************************************************
		 * IFM_TILE
		 ************************************************************/
		on tile[IFM_TILE]:
		{

			/* QEI Service Loop */
			{
			    QEIConfig qei_config;
                    qei_config.index = QEI_WITH_INDEX;                  // Indexed encoder
                    qei_config.real_counts = 16000;                     // 4 x 4000 ticks (Cuadrature encoder)
                    qei_config.sensor_polarity = QEI_POLARITY_NORMAL;   // CW
                    qei_config.poles = 4;                               // 4 pole pairs on motor (for hall syncronization, not always used)
                    qei_config.max_ticks = 10 * qei_config.real_counts; // 10 turns

				qei_service(i_qei, qei_ports, qei_config);
			}
		}
	}

	return 0;
}
