/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.inc>
#include <IFM_DC100-rev-b.inc>

/**
 * @file test_hall.xc
 * @brief Test illustrates usage of hall sensor to get position and velocity information
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <hall_service.h>

/* Test Hall Sensor Client */
void hall_test(interface HallInterface client i_hall)
{
    int position = 0;
    int velocity = 0;
    int count = 0;
    int direction;
    int pins = 0;

    while(1)
    {
        /* get position from Hall Sensor */
        {count, direction} = i_hall.get_hall_position_absolute();
        position = i_hall.get_hall_position();

        /* get velocity from Hall Sensor */
        velocity = i_hall.get_hall_velocity();

        /* get pins state from Hall Sensor */
        pins = i_hall.get_hall_pinstate();

        xscope_int(COUNT, count);
        xscope_int(POSITION, position);
        xscope_int(VELOCITY, velocity);
        xscope_int(A, (pins&1)*1000); // scale to 1000 for easier display in xscope
        xscope_int(B, (pins&0b10)*500);
        xscope_int(C, (pins&0b100)*250);

    }
}

HallPorts hall_ports = SOMANET_IFM_HALL_PORTS;

int main(void)
{
    interface HallInterface i_hall[5];

    par
    {
        on tile[APP_TILE]: hall_test(i_hall[0]);

        on tile[IFM_TILE]:
        {
            HallConfig hall_config;
            hall_config.pole_pairs = POLE_PAIRS;

            hall_service(hall_ports, hall_config, i_hall);
        }
    }

    return 0;
}
