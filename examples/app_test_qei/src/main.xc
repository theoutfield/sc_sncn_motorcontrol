/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IMF_BOARD_REQUIRED" WIT A APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.bsp>
#include <IFM_DC100-rev-b.bsp>

/**
 * @file test_qei.xc
 * @brief Test illustrates usage of qei sensor to get position and velocity information
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <qei_service.h>

/* Test QEI Sensor Client */
void qei_test(interface QEIInterface client i_qei)
{
    int position, velocity, count, valid;

    while(1)
    {
        /* get position and velocity from QEI Sensor */
        count = i_qei.get_qei_position_absolute();
        {position, valid} = i_qei.get_qei_position();

        velocity = i_qei.get_qei_velocity();

        xscope_int(POSITION, position);
        xscope_int(VELOCITY, velocity);

        //printf("Position: %d Velocity: %d\n", position, velocity);

        delay_milliseconds(1);
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

        /***************************************************
         * IFM TILE
         ***************************************************/
        on tile[IFM_TILE]:
        /* Quadrature Encoder sensor Service */
        {
            QEIConfig qei_config;
            qei_config.signal_type = QEI_RS422_SIGNAL;              // Encoder signal type
            qei_config.index_type = QEI_WITH_INDEX;                 // Indexed encoder?
            qei_config.ticks_resolution = 4000;                     // Encoder resolution
            qei_config.sensor_polarity = QEI_POLARITY_NORMAL;       // CW

            qei_service(qei_ports, qei_config, i_qei);
        }
    }

    return 0;
}
